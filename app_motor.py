"""
Smart Motor Control - PMC-2HSP 모터 제어 앱
4개 모터 (상부 스테이지/회전, 하부 스테이지/회전) 스케줄 제어
"""
import flet as ft
import asyncio
import base64
import io
import json
import sys
import os
import time

sys.path.insert(0, os.path.dirname(__file__))

from devices.motor_controller import (
    MotorController, PULSE_PER_MM, STEP_ANGLE,
    speed_pps_to_mm_per_sec, speed_pps_to_deg_per_sec,
    mm_to_pulse, degree_to_pulse,
)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

try:
    import serial.tools.list_ports
except ImportError:
    serial = None

MOTOR_IDS = ['upper_stage', 'upper_rotate', 'lower_stage', 'lower_rotate']
MOTOR_LABELS = ['상부 스테이지', '상부 회전', '하부 스테이지', '하부 회전']
MOTOR_SHORT = ['U-Stage', 'U-Rotate', 'L-Stage', 'L-Rotate']
MOTOR_TYPES = ['linear', 'rotate', 'linear', 'rotate']
DIRECTION_OPTIONS = {
    'linear': ['+', '-'],
    'rotate': ['CW', 'CCW'],
}
DIRECTION_MAP = {'+': 'plus', '-': 'minus', 'CW': 'cw', 'CCW': 'ccw'}

# 스테이지 물리 제한: 모터간 43.5cm, 각 모터 0~45cm 이동 가능 (초기=하단)
STAGE_GAP_MM = 435.0
MAX_TRAVEL_MM = 450.0


class MotorApp:

    def __init__(self):
        self.motor_ctrl: MotorController | None = None
        self.selected_motor = 0
        self._conn_status_value = "Disconnected"
        self._conn_status_color = "red"

        self.steps = [
            [{'speed': None, 'dur': None, 'dir_dd': None, 'dist_text': None} for _ in range(8)]
            for _ in range(4)
        ]
        self.step_enabled = [
            # Safer default: only S1 enabled, S2+ are opt-in.
            [True, False, False, False, False, False, False, False]
            for _ in range(4)
        ]

        self.schedule_running = False
        self.schedule_time = 0.0
        self.history = [[] for _ in range(4)]
        self._schedule_cmd_dt = 0.1          # command update period (s)
        self._graph_render_interval = 1.0    # graph render period (s)
        self._position_dt = 0.1              # position integration period (s)
        self._last_schedule_cmd = [{"spd": None, "dir": None} for _ in range(4)]
        self._schedule_relative_mode = True  # 스케줄 상대위치 방식

        # 절대좌표 추적
        self._config_path = os.path.join(os.path.dirname(__file__), 'motor_home.json')
        self.stage_gap = STAGE_GAP_MM
        self.cur_z = {'upper': 0.0, 'lower': 0.0}
        self.cur_angle = {'upper': 0.0, 'lower': 0.0}
        self.home_z = {'upper': 0.0, 'lower': 0.0}
        self.home_angle = {'upper': 0.0, 'lower': 0.0}
        self._homing = False
        self._graph_loop_running = False
        self._limit_alarm_triggered = False
        self._limit_alarm_dialog = None
        self._load_home()

        self.motor_mode = "schedule"  # "schedule" or "manual"
        self.manual_speeds = [
            {"label": "very slow", "pps": 5},
            {"label": "slow", "pps": 50},
            {"label": "fast", "pps": 300},
        ]
        self.speed_mode_idx = 0
        self._last_applied_manual_speed_pps = None
        self.motor_running = [False, False, False, False]
        self._manual_pressed_at = [0.0, 0.0, 0.0, 0.0]
        self.motor_manual_dir = ['+', 'CW', '+', 'CW']
        self._motor_labels = [None, None, None, None]
        self._motor_rows = [None, None, None, None]
        self.speed_unit = "mm/s"  # "mm/s", "cm/s", "m/s"

    # ──────────────────────────────────────────────
    # Home position persistence
    # ──────────────────────────────────────────────

    def _load_home(self):
        try:
            with open(self._config_path, 'r') as f:
                data = json.load(f)
            self.home_z = data.get('home_z', {'upper': 0.0, 'lower': 0.0})
            self.home_angle = data.get('home_angle', {'upper': 0.0, 'lower': 0.0})
            self.stage_gap = data.get('stage_gap', STAGE_GAP_MM)

            for k in ('upper', 'lower'):
                self.home_z[k] = max(0, min(MAX_TRAVEL_MM, self.home_z.get(k, 0)))

            # 마지막 위치 복원 (없으면 홈 위치로)
            last_z = data.get('last_z', self.home_z)
            last_angle = data.get('last_angle', self.home_angle)
            self.cur_z = {
                'upper': max(0, min(MAX_TRAVEL_MM, last_z.get('upper', self.home_z['upper']))),
                'lower': max(0, min(MAX_TRAVEL_MM, last_z.get('lower', self.home_z['lower']))),
            }
            self.cur_angle = {
                'upper': last_angle.get('upper', self.home_angle['upper']),
                'lower': last_angle.get('lower', self.home_angle['lower']),
            }
            print(f"[Position] restored: z={self.cur_z}, angle={self.cur_angle}")
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            pass

    def _check_stage_limits(self) -> tuple[bool, str]:
        """스테이지 위치가 0~MAX_TRAVEL_MM 범위 내인지 확인. (ok, msg) 반환."""
        for key, label in [('upper', '상부'), ('lower', '하부')]:
            z = self.cur_z[key]
            if z < 0:
                return False, f"{label} 스테이지 하한 초과 ({z:.1f}mm < 0mm)"
            if z > MAX_TRAVEL_MM:
                return False, f"{label} 스테이지 상한 초과 ({z:.1f}mm > {MAX_TRAVEL_MM:.0f}mm)"
        return True, ""

    def _trigger_limit_alarm(self, msg: str):
        """범위 초과 시 모든 모터 정지 + 알람 다이얼로그"""
        if self._limit_alarm_triggered:
            return
        self._limit_alarm_triggered = True
        self.schedule_running = False
        self._homing = False
        for i in range(4):
            self.motor_running[i] = False
            self._highlight_motor(i, False)
        if self.motor_ctrl and self.motor_ctrl.connected:
            self.motor_ctrl.stop_all(immediate=False)
        for k in ('upper', 'lower'):
            self.cur_z[k] = max(0, min(MAX_TRAVEL_MM, self.cur_z[k]))
        if hasattr(self, '_status_text') and self._status_text:
            self._status_text.value = f"⚠ 한계 초과: {msg}"
            self._status_text.color = "#ff0000"
        if hasattr(self, 'start_btn') and self.start_btn:
            self.start_btn.text = "Start"
        self._update_all_graphs()
        if hasattr(self, 'page') and self.page:
            self._limit_alarm_dialog = ft.AlertDialog(
                modal=True,
                title=ft.Text("스테이지 한계 초과", color="#ff0000", weight=ft.FontWeight.BOLD),
                content=ft.Text(f"{msg}\n\n모든 모터가 정지되었습니다.", size=14),
                on_dismiss=lambda e: setattr(self, '_limit_alarm_triggered', False),
            )
            self.page.overlay.append(self._limit_alarm_dialog)
            self._limit_alarm_dialog.open = True
            self.page.update()

    def _would_exceed_limit(self, motor_idx: int, direction: str) -> bool:
        """해당 방향 이동 시 한계 초과 여부 (수동 조작 전 체크용)"""
        if MOTOR_TYPES[motor_idx] != 'linear':
            return False
        key = 'upper' if motor_idx == 0 else 'lower'
        z = self.cur_z[key]
        if direction in ('+', 'plus', 'up'):
            return z >= MAX_TRAVEL_MM
        if direction in ('-', 'minus', 'down'):
            return z <= 0
        return False

    def _save_home(self):
        data = {
            'home_z': self.home_z,
            'home_angle': self.home_angle,
            'stage_gap': self.stage_gap,
            'last_z': self.cur_z,
            'last_angle': self.cur_angle,
        }
        try:
            with open(self._config_path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as ex:
            print(f"Home save error: {ex}")

    def _save_last_position(self):
        """현재 위치만 JSON에 업데이트 (앱 종료 시 호출)"""
        try:
            try:
                with open(self._config_path, 'r') as f:
                    data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                data = {}
            data['last_z'] = self.cur_z
            data['last_angle'] = self.cur_angle
            with open(self._config_path, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"[Position] saved: z={self.cur_z}, angle={self.cur_angle}")
        except Exception as ex:
            print(f"Position save error: {ex}")

    # ──────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────

    @staticmethod
    def _parse_duration_to_hours(text: str) -> float:
        """HH:MM:SS 또는 초 단위 숫자 → 시간(hours)"""
        text = (text or "0").strip()
        if ":" in text:
            parts = text.split(":")
            try:
                h = int(parts[0]) if len(parts) > 0 and parts[0] else 0
                m = int(parts[1]) if len(parts) > 1 and parts[1] else 0
                s = int(parts[2]) if len(parts) > 2 and parts[2] else 0
                return h + m / 60.0 + s / 3600.0
            except ValueError:
                return 0.0
        try:
            # "5" 등 숫자만 입력 시 → 초 단위로 해석 (5 = 5초)
            sec = float(text)
            return sec / 3600.0
        except ValueError:
            return 0.0

    @staticmethod
    def _calc_distance(speed_pps: int, dur_hours: float, motor_type: str) -> str:
        seconds = dur_hours * 3600
        if motor_type == 'linear':
            dist_mm = speed_pps_to_mm_per_sec(speed_pps) * seconds
            if dist_mm >= 1.0:
                return f"{dist_mm:.2f} mm"
            return f"{dist_mm * 1000:.1f} μm"
        else:
            # PPS → RPM → 총 회전수
            rpm = speed_pps_to_deg_per_sec(speed_pps) * 60.0 / 360.0
            revs = rpm * (seconds / 60.0)
            return f"{revs:.2f} rev"

    # ──────────────────────────────────────────────
    # UI Components (table inputs, keypads)
    # ──────────────────────────────────────────────

    def _make_table_input(self, value="0", w=80, h=34, on_change=None):
        display = ft.Text(value, size=12, text_align=ft.TextAlign.CENTER)
        cell = ft.Container(
            content=display, width=w, height=h,
            alignment=ft.Alignment(0, 0), bgcolor="#ffffff",
            border_radius=2, on_click=lambda e: self._show_numpad(cell),
        )
        cell.value = value
        cell.on_change = on_change
        cell._display = display
        cell._is_duration = False
        return cell

    def _make_duration_input(self, value="00:00:30", w=90, h=34, on_change=None):
        display = ft.Text(value, size=12, text_align=ft.TextAlign.CENTER)
        cell = ft.Container(
            content=display, width=w, height=h,
            alignment=ft.Alignment(0, 0), bgcolor="#ffffff",
            border_radius=2, on_click=lambda e: self._show_duration_pad(cell),
        )
        cell.value = value
        cell.on_change = on_change
        cell._display = display
        cell._is_duration = True
        return cell

    # ── Numpad ──

    def _build_numpad(self):
        self._numpad_target = None
        self._numpad_value = ""
        self._numpad_open = False
        self._numpad_display = ft.Text("0", size=32, weight=ft.FontWeight.BOLD,
                                       text_align=ft.TextAlign.RIGHT)

        def _kb(label, w=70, bg="#f5f5f5", fg="#000000"):
            return ft.Container(
                content=ft.Text(label, size=22, weight=ft.FontWeight.BOLD,
                                color=fg, text_align=ft.TextAlign.CENTER),
                width=w, height=60, bgcolor=bg, border_radius=8,
                alignment=ft.Alignment(0, 0), border=ft.Border.all(1, "#cccccc"),
                on_click=lambda e, k=label: self._numpad_key(k),
            )

        ok_btn = ft.Container(
            content=ft.Text("OK", size=22, weight=ft.FontWeight.BOLD,
                            color="#ffffff", text_align=ft.TextAlign.CENTER),
            width=70, height=60, bgcolor="#003366", border_radius=8,
            alignment=ft.Alignment(0, 0), on_click=lambda e: self._numpad_confirm(),
        )
        cancel_btn = ft.Container(
            content=ft.Text("Cancel", size=16, color="#666666",
                            text_align=ft.TextAlign.CENTER),
            width=310, height=40, bgcolor="#eeeeee", border_radius=8,
            alignment=ft.Alignment(0, 0), on_click=lambda e: self._numpad_cancel(),
        )
        pad_card = ft.Container(
            content=ft.Column([
                ft.Text("숫자 입력", size=18, weight=ft.FontWeight.BOLD),
                ft.Container(height=4),
                ft.Container(content=self._numpad_display, bgcolor="#f9f9f9",
                             border=ft.Border.all(2, "#003366"), border_radius=8,
                             padding=12, width=310, alignment=ft.Alignment(1, 0)),
                ft.Container(height=8),
                ft.Row([_kb("7"), _kb("8"), _kb("9"), _kb("\u232b", bg="#ffe0e0")],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Row([_kb("4"), _kb("5"), _kb("6"), _kb("C", bg="#ffe0e0")],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Row([_kb("1"), _kb("2"), _kb("3"), _kb("\u00b1", bg="#e0e0ff")],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Row([_kb("0", w=148), _kb("."), ok_btn],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Container(height=4), cancel_btn,
            ], spacing=8, horizontal_alignment=ft.CrossAxisAlignment.CENTER),
            width=360, bgcolor="#ffffff", border_radius=12,
            padding=20, border=ft.Border.all(2, "#003366"),
        )
        self._numpad_overlay = ft.Container(
            content=pad_card, bgcolor="rgba(0,0,0,0.4)",
            alignment=ft.Alignment(0, 0), expand=True, visible=False,
            on_click=lambda e: self._numpad_cancel(),
        )

    def _show_numpad(self, target):
        if self._numpad_open:
            return
        self._numpad_open = True
        self._numpad_target = target
        self._numpad_value = target.value or ""
        self._numpad_display.value = self._numpad_value or "0"
        self._numpad_overlay.visible = True
        self.page.update()

    def _numpad_key(self, key):
        if key == "\u232b":
            self._numpad_value = self._numpad_value[:-1]
        elif key == "C":
            self._numpad_value = ""
        elif key == "\u00b1":
            if self._numpad_value.startswith("-"):
                self._numpad_value = self._numpad_value[1:]
            elif self._numpad_value:
                self._numpad_value = "-" + self._numpad_value
        elif key == ".":
            if "." not in self._numpad_value:
                self._numpad_value = (self._numpad_value or "0") + "."
        else:
            self._numpad_value += key
        self._numpad_display.value = self._numpad_value or "0"
        self.page.update()

    def _numpad_confirm(self):
        if self._numpad_target:
            val = self._numpad_value or "0"
            self._numpad_target.value = val
            if hasattr(self._numpad_target, '_display'):
                self._numpad_target._display.value = val
            if hasattr(self._numpad_target, 'on_change') and self._numpad_target.on_change:
                try:
                    self._numpad_target.on_change(None)
                except Exception:
                    pass
        self._numpad_open = False
        self._numpad_overlay.visible = False
        self.page.update()

    def _numpad_cancel(self):
        self._numpad_open = False
        self._numpad_overlay.visible = False
        self.page.update()

    # ── Duration Pad ──

    def _build_duration_pad(self):
        self._durpad_target = None
        self._durpad_value = ""
        self._durpad_open = False
        self._durpad_display = ft.Text("00:00:00", size=32, weight=ft.FontWeight.BOLD,
                                       text_align=ft.TextAlign.RIGHT)

        def _kb(label, w=70, bg="#f5f5f5", fg="#000000"):
            return ft.Container(
                content=ft.Text(label, size=22, weight=ft.FontWeight.BOLD,
                                color=fg, text_align=ft.TextAlign.CENTER),
                width=w, height=60, bgcolor=bg, border_radius=8,
                alignment=ft.Alignment(0, 0), border=ft.Border.all(1, "#cccccc"),
                on_click=lambda e, k=label: self._durpad_key(k),
            )

        ok_btn = ft.Container(
            content=ft.Text("OK", size=22, weight=ft.FontWeight.BOLD,
                            color="#ffffff", text_align=ft.TextAlign.CENTER),
            width=70, height=60, bgcolor="#003366", border_radius=8,
            alignment=ft.Alignment(0, 0), on_click=lambda e: self._durpad_confirm(),
        )
        cancel_btn = ft.Container(
            content=ft.Text("Cancel", size=16, color="#666666",
                            text_align=ft.TextAlign.CENTER),
            width=310, height=40, bgcolor="#eeeeee", border_radius=8,
            alignment=ft.Alignment(0, 0), on_click=lambda e: self._durpad_cancel(),
        )
        pad_card = ft.Container(
            content=ft.Column([
                ft.Text("시간 입력 (HH:MM:SS)", size=18, weight=ft.FontWeight.BOLD),
                ft.Container(height=4),
                ft.Container(content=self._durpad_display, bgcolor="#f9f9f9",
                             border=ft.Border.all(2, "#003366"), border_radius=8,
                             padding=12, width=310, alignment=ft.Alignment(1, 0)),
                ft.Container(height=8),
                ft.Row([_kb("7"), _kb("8"), _kb("9"), _kb("\u232b", bg="#ffe0e0")],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Row([_kb("4"), _kb("5"), _kb("6"), _kb("C", bg="#ffe0e0")],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Row([_kb("1"), _kb("2"), _kb("3"), _kb(":", bg="#e0e0ff")],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Row([_kb("0", w=148), _kb("00"), ok_btn],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Container(height=4), cancel_btn,
            ], spacing=8, horizontal_alignment=ft.CrossAxisAlignment.CENTER),
            width=360, bgcolor="#ffffff", border_radius=12,
            padding=20, border=ft.Border.all(2, "#003366"),
        )
        self._durpad_overlay = ft.Container(
            content=pad_card, bgcolor="rgba(0,0,0,0.4)",
            alignment=ft.Alignment(0, 0), expand=True, visible=False,
            on_click=lambda e: self._durpad_cancel(),
        )

    def _format_duration(self, raw: str) -> str:
        digits = raw.replace(":", "")
        digits = digits.lstrip("0") or "0"
        digits = digits.zfill(6)
        if len(digits) > 6:
            digits = digits[-6:]
        return f"{digits[0:2]}:{digits[2:4]}:{digits[4:6]}"

    def _show_duration_pad(self, target):
        if self._durpad_open:
            return
        self._durpad_open = True
        self._durpad_target = target
        cur = (target.value or "00:00:00").replace(":", "")
        self._durpad_value = cur
        self._durpad_display.value = self._format_duration(cur)
        self._durpad_overlay.visible = True
        self.page.update()

    def _durpad_key(self, key):
        if key == "\u232b":
            self._durpad_value = self._durpad_value[:-1]
        elif key == "C":
            self._durpad_value = ""
        elif key == ":":
            pass
        elif key == "00":
            self._durpad_value += "00"
        else:
            self._durpad_value += key
        self._durpad_display.value = self._format_duration(self._durpad_value)
        self.page.update()

    def _durpad_confirm(self):
        if self._durpad_target:
            val = self._format_duration(self._durpad_value)
            self._durpad_target.value = val
            if hasattr(self._durpad_target, '_display'):
                self._durpad_target._display.value = val
            if hasattr(self._durpad_target, 'on_change') and self._durpad_target.on_change:
                try:
                    self._durpad_target.on_change(None)
                except Exception:
                    pass
        self._durpad_open = False
        self._durpad_overlay.visible = False
        self.page.update()

    def _durpad_cancel(self):
        self._durpad_open = False
        self._durpad_overlay.visible = False
        self.page.update()

    # ──────────────────────────────────────────────
    # Table builder (same style as gas/temp project)
    # ──────────────────────────────────────────────

    def _build_table(self, headers, data_rows, col_widths, row_height=34):
        border_color = "#999999"
        rows_controls = []
        header_cells = []
        for i, h in enumerate(headers):
            header_cells.append(ft.Container(
                content=ft.Text(h, size=11, weight=ft.FontWeight.BOLD,
                                text_align=ft.TextAlign.CENTER, color="#ffffff"),
                width=col_widths[i], height=row_height,
                bgcolor="#003366", alignment=ft.Alignment(0, 0),
                border=ft.Border(
                    left=ft.BorderSide(1, border_color) if i > 0 else ft.BorderSide(0, "transparent"),
                    right=ft.BorderSide(0, "transparent"),
                    top=ft.BorderSide(0, "transparent"),
                    bottom=ft.BorderSide(1, border_color),
                ),
            ))
        rows_controls.append(ft.Row(header_cells, spacing=0))

        for r_idx, row_data in enumerate(data_rows):
            cells = []
            for c_idx, cell_content in enumerate(row_data):
                if isinstance(cell_content, ft.Control):
                    wrapper = ft.Container(
                        content=cell_content, width=col_widths[c_idx], height=row_height,
                        alignment=ft.Alignment(0, 0),
                        border=ft.Border(
                            left=ft.BorderSide(1, border_color) if c_idx > 0 else ft.BorderSide(0, "transparent"),
                            right=ft.BorderSide(0, "transparent"),
                            top=ft.BorderSide(0, "transparent"),
                            bottom=ft.BorderSide(1, border_color),
                        ),
                    )
                else:
                    wrapper = ft.Container(
                        content=ft.Text(str(cell_content), size=11, text_align=ft.TextAlign.CENTER),
                        width=col_widths[c_idx], height=row_height,
                        alignment=ft.Alignment(0, 0),
                        border=ft.Border(
                            left=ft.BorderSide(1, border_color) if c_idx > 0 else ft.BorderSide(0, "transparent"),
                            right=ft.BorderSide(0, "transparent"),
                            top=ft.BorderSide(0, "transparent"),
                            bottom=ft.BorderSide(1, border_color),
                        ),
                    )
                cells.append(wrapper)
            rows_controls.append(ft.Row(cells, spacing=0))

        return ft.Container(
            content=ft.Column(rows_controls, spacing=0),
            border=ft.Border.all(1, border_color), border_radius=4,
        )

    def _make_step_toggle(self, label, enabled, on_click):
        bg = "#003366" if enabled else "#cccccc"
        fg = "#ffffff" if enabled else "#333333"
        return ft.Container(
            content=ft.Text(label, size=11, weight=ft.FontWeight.BOLD,
                            color=fg, text_align=ft.TextAlign.CENTER),
            width=46, height=28, bgcolor=bg, border_radius=4,
            alignment=ft.Alignment(0, 0), on_click=on_click,
        )

    # ──────────────────────────────────────────────
    # Settings panel
    # ──────────────────────────────────────────────

    def _build_settings(self):
        self._port_dropdown = ft.Dropdown(
            width=200, value="/dev/ttyS1", text_size=13, dense=True,
            options=self._scan_ports(),
        )
        self._baud_dropdown = ft.Dropdown(
            width=120, value="9600", text_size=13, dense=True,
            options=[
                ft.DropdownOption("9600"),
                ft.DropdownOption("19200"),
                ft.DropdownOption("38400"),
            ],
        )

        self._conn_status = ft.Text(self._conn_status_value, color=self._conn_status_color, size=13)

        self._gap_input = self._make_table_input(str(self.stage_gap), w=100, h=34)
        gap_apply_btn = ft.ElevatedButton("Apply", bgcolor="#2196F3", color="white",
                                          on_click=lambda e: self._apply_gap())

        self._parity_label = ft.Text("Parity: N (fixed)", size=13, color="#888888")
        self._rs485_label = ft.Text("RS-485: fixed", size=13, color="#888888")

        refresh_btn = ft.ElevatedButton("Refresh Ports", on_click=lambda e: self._refresh_ports())
        connect_btn = ft.ElevatedButton("Connect", bgcolor="#4CAF50", color="white",
                                        on_click=lambda e: self._connect_motor())
        disconnect_btn = ft.ElevatedButton("Disconnect", bgcolor="#f44336", color="white",
                                           on_click=lambda e: self._disconnect_motor())
        diagnose_btn = ft.ElevatedButton("Diagnose", bgcolor="#FF9800", color="white",
                                         on_click=lambda e: self._diagnose_serial())
        mode_btn = ft.ElevatedButton("Check Mode", bgcolor="#9C27B0", color="white",
                                     on_click=lambda e: self._check_operating_mode())
        stop_all_btn = ft.ElevatedButton("STOP ALL", bgcolor="#ff0000", color="white",
                                         on_click=lambda e: self._confirm_emergency_stop())

        reset_to_bottom_btn = ft.ElevatedButton(
            "모터 최하단 위치로 초기화",
            bgcolor="#607D8B", color="white",
            on_click=lambda e: self._confirm_reset_to_bottom(),
        )

        self._diag_text = ft.Text("", size=11, selectable=True, color="#888888")

        return ft.Container(
            content=ft.Column([
                ft.Text("Motor Settings", size=16, weight=ft.FontWeight.BOLD),
                ft.Divider(),
                ft.Row([ft.Text("Port:", width=80), self._port_dropdown, refresh_btn]),
                ft.Row([ft.Text("Baud:", width=80), self._baud_dropdown,
                        self._parity_label, self._rs485_label]),
                ft.Row([connect_btn, disconnect_btn, diagnose_btn, mode_btn]),
                self._conn_status,
                ft.Divider(),
                ft.Text("Stage Configuration", size=14, weight=ft.FontWeight.BOLD),
                ft.Row([ft.Text("Gap (mm):", width=80), self._gap_input, gap_apply_btn]),
                ft.Divider(),
                ft.Text("Position Reset", size=14, weight=ft.FontWeight.BOLD),
                ft.Text("두 모터가 물리적으로 최하단에 있을 때 사용", size=11, color="#888888"),
                reset_to_bottom_btn,
                ft.Divider(),
                stop_all_btn,
                self._diag_text,
            ], spacing=10),
            padding=15,
        )

    def _confirm_reset_to_bottom(self):
        def _do_reset(e):
            dlg.open = False
            self.page.update()
            self._reset_to_bottom()

        def _cancel(e):
            dlg.open = False
            self.page.update()

        dlg = ft.AlertDialog(
            modal=True,
            title=ft.Text("최하단 위치 초기화", weight=ft.FontWeight.BOLD),
            content=ft.Text(
                "두 스테이지 모터가 현재 물리적으로\n최하단(0mm)에 위치해 있습니까?\n\n"
                "확인 시 소프트웨어 위치가 0mm로 초기화됩니다.",
                size=13,
            ),
            actions=[
                ft.TextButton("취소", on_click=_cancel),
                ft.ElevatedButton("초기화", bgcolor="#607D8B", color="white", on_click=_do_reset),
            ],
            actions_alignment=ft.MainAxisAlignment.END,
        )
        self.page.overlay.append(dlg)
        dlg.open = True
        self.page.update()

    def _reset_to_bottom(self):
        """두 스테이지 모터가 최하단에 있다는 전제로 소프트웨어 위치를 0으로 초기화"""
        self.cur_z = {'upper': 0.0, 'lower': 0.0}
        self.cur_angle = {'upper': 0.0, 'lower': 0.0}
        self.home_z = {'upper': 0.0, 'lower': 0.0}
        self.home_angle = {'upper': 0.0, 'lower': 0.0}
        self.stage_gap = STAGE_GAP_MM
        if hasattr(self, '_gap_input') and self._gap_input:
            self._gap_input.value = str(self.stage_gap)
            self._gap_input._display.value = str(self.stage_gap)
        # 드라이버 위치 카운터도 클리어
        if self.motor_ctrl and self.motor_ctrl.connected:
            try:
                self.motor_ctrl.clear_position_counter_all()
            except Exception as ex:
                print(f"[ResetToBottom] clear_position_counter_all error: {ex}")
        self._save_home()
        self._update_all_graphs()
        if hasattr(self, '_status_text') and self._status_text:
            self._status_text.value = "위치 초기화 완료 (최하단 = 0mm)"
            self._status_text.color = "#607D8B"
        self.page.update()

    def _apply_gap(self):
        try:
            self.stage_gap = float(self._gap_input.value)
        except (ValueError, TypeError):
            self.stage_gap = STAGE_GAP_MM
        self._save_home()
        self._update_all_graphs()
        self.page.update()

    def _scan_ports(self):
        default_ports = ["COM7", "/dev/ttyS1"]
        if serial:
            ports = serial.tools.list_ports.comports()
            devices = [p.device for p in ports]
            for dp in default_ports:
                if dp not in devices:
                    devices.append(dp)
            return [ft.DropdownOption(d) for d in devices]
        return [ft.DropdownOption(p) for p in default_ports]

    def _refresh_ports(self):
        self._port_dropdown.options = self._scan_ports()
        self.page.update()

    def _connect_motor(self):
        port = self._port_dropdown.value or "/dev/ttyS1"
        try:
            baud = int(self._baud_dropdown.value)
        except (ValueError, TypeError):
            baud = 9600
        if baud not in (9600, 19200, 38400):
            baud = 9600
            self._baud_dropdown.value = "9600"
        self.motor_ctrl = MotorController(port=port, baudrate=baud, parity='N', rs485_mode=True)
        ok = self.motor_ctrl.connect()
        if ok:
            verify = self.motor_ctrl.verify_connection()
            v_str = " | ".join(f"D{k}:{v}" for k, v in verify.items())
            all_ok = all("OK" in str(v) for v in verify.values())
            if all_ok:
                init_speed = self.manual_speeds[self.speed_mode_idx]["pps"]
                self.motor_ctrl.set_speed_all(init_speed)
                self._last_applied_manual_speed_pps = init_speed
                print(f"[Init] speed_ratio set to {init_speed} PPS")
                self._set_conn_status(f"Connected ({port} @ {baud}) [{v_str}]", "#4CAF50")
            else:
                self._set_conn_status(f"Port open but driver error [{v_str}]", "#FF9800")
        else:
            self._set_conn_status(f"Failed ({port} @ {baud})", "red")
            self.motor_ctrl = None
        self.page.update()

    def _check_operating_mode(self):
        """PMC-2HSP 동작 모드 확인 (Func 02 MODE0/MODE1)"""
        if not self.motor_ctrl or not self.motor_ctrl.connected:
            self._diag_text.value = "Connect first"
            self.page.update()
            return
        m1 = self.motor_ctrl.get_operating_mode(1)
        m2 = self.motor_ctrl.get_operating_mode(2)
        msg = f"Driver1: {m1 or 'N/A'}  |  Driver2: {m2 or 'N/A'}"
        if m1 or m2:
            msg += "\n(Jog=수동, Continuous=연속, Index=인덱스, Program=프로그램)"
        self._diag_text.value = msg
        self.page.update()

    def _set_conn_status(self, value: str, color: str):
        self._conn_status_value = value
        self._conn_status_color = color
        if hasattr(self, "_conn_status") and self._conn_status:
            self._conn_status.value = value
            self._conn_status.color = color

    async def _auto_connect_on_start(self):
        await asyncio.sleep(0.2)
        port = "/dev/ttyS1"
        try:
            if hasattr(self, "_port_dropdown") and self._port_dropdown.value:
                port = self._port_dropdown.value
        except Exception:
            pass

        for baud in (9600, 19200, 38400):
            ctrl = MotorController(port=port, baudrate=baud, parity='N', rs485_mode=True)
            if not ctrl.connect():
                continue
            verify = ctrl.verify_connection()
            all_ok = all("OK" in str(v) for v in verify.values())
            if all_ok:
                self.motor_ctrl = ctrl
                self._last_applied_manual_speed_pps = self.manual_speeds[self.speed_mode_idx]["pps"]
                self.motor_ctrl.set_speed_all(self._last_applied_manual_speed_pps)
                self._set_conn_status(f"Connected ({port} @ {baud})", "#4CAF50")
                try:
                    if hasattr(self, "_baud_dropdown"):
                        self._baud_dropdown.value = str(baud)
                except Exception:
                    pass
                self.page.update()
                return
            ctrl.disconnect()

        self._set_conn_status(f"Failed ({port})", "red")
        self.page.update()

    def _diagnose_serial(self):
        """시리얼 진단: 포트 1회 open, parity/RS-485 순차 테스트"""
        import serial as pyserial
        import struct
        import time as _t

        port = self._port_dropdown.value or "/dev/ttyS1"
        try:
            baud = int(self._baud_dropdown.value)
        except (ValueError, TypeError):
            baud = 9600

        lines = [f"=== Diagnose {port} @ {baud} ==="]

        try:
            ser = pyserial.Serial(port=port, baudrate=baud,
                                  parity=pyserial.PARITY_NONE,
                                  stopbits=1, bytesize=8, timeout=1)
        except Exception as e:
            lines.append(f"Port open FAIL: {e}")
            self._diag_text.value = "\n".join(lines)
            self.page.update()
            return

        lines.append(f"Port open OK: {ser.name}")
        lines.append(f"  fd={ser.fileno()}, rts={ser.rts}, dtr={ser.dtr}")

        def _send_and_read(slave_id):
            req = struct.pack('>BBH H', slave_id, 0x03, 0x0000, 0x0001)
            crc = self._modbus_crc(req)
            req += struct.pack('<H', crc)
            ser.reset_input_buffer()
            _t.sleep(0.05)
            ser.write(req)
            ser.flush()
            _t.sleep(0.5)
            n = ser.in_waiting
            resp = ser.read(n) if n > 0 else b''
            return req, resp

        def _valid(resp, slave_id):
            if len(resp) >= 5 and resp[0] == slave_id and resp[1] == 0x03:
                return "OK"
            if len(resp) >= 3 and resp[0] == slave_id and resp[1] == 0x83:
                return f"Exception(code={resp[2]})"
            return None

        best = None

        # --- Test 1: RS-485 OFF ---
        lines.append(f"\n[Test 1] Parity=N, RS485=OFF")
        for sid in (1, 2):
            req, resp = _send_and_read(sid)
            hx = resp.hex(' ').upper() if resp else "(none)"
            lines.append(f"  S{sid} TX={req.hex(' ').upper()}")
            lines.append(f"     RX={hx} ({len(resp)}B)")
            v = _valid(resp, sid)
            if v:
                lines.append(f"     ✓ {v}")
                if not best:
                    best = ("RS485=OFF", False)

        # --- Test 2: RS-485 ON ---
        rs485_ok = False
        try:
            import serial.rs485
            ser.rs485_mode = serial.rs485.RS485Settings(
                rts_level_for_tx=True, rts_level_for_rx=False,
                delay_before_tx=0.0, delay_before_rx=0.005)
            rs485_ok = True
        except Exception as e:
            lines.append(f"\n[RS-485 mode FAIL: {e}]")

        if rs485_ok:
            lines.append(f"\n[Test 2] Parity=N, RS485=ON")
            _t.sleep(0.1)
            for sid in (1, 2):
                req, resp = _send_and_read(sid)
                hx = resp.hex(' ').upper() if resp else "(none)"
                lines.append(f"  S{sid} TX={req.hex(' ').upper()}")
                lines.append(f"     RX={hx} ({len(resp)}B)")
                v = _valid(resp, sid)
                if v:
                    lines.append(f"     ✓ {v}")
                    if not best:
                        best = ("RS485=ON", True)

        # --- Test 3: Loopback ---
        lines.append(f"\n[Test 3] Loopback")
        try:
            ser.rs485_mode = None
        except Exception:
            pass
        _t.sleep(0.1)
        ser.reset_input_buffer()
        test_data = b'\xAA\x55\x01\x02'
        ser.write(test_data)
        ser.flush()
        _t.sleep(0.3)
        echo = ser.read(ser.in_waiting or 0)
        if echo == test_data:
            lines.append(f"  ✓ Echo match (TX→RX 루프백 있음)")
        elif echo:
            lines.append(f"  Partial: {echo.hex(' ').upper()} ({len(echo)}B)")
        else:
            lines.append(f"  No echo")

        ser.close()

        # --- 결과 ---
        lines.append(f"\n{'=' * 40}")
        if best:
            cfg, needs_rs485 = best
            lines.append(f"★ 통신 성공! {cfg}")
            if needs_rs485:
                lines.append(f"  → RS-485 mode 체크 후 Connect")
        else:
            lines.append("✗ Modbus 응답 없음")
            lines.append("")
            lines.append("확인:")
            lines.append("  1. 드라이버 전원 ON?")
            lines.append("  2. RS-485 A/B 배선")
            lines.append("  3. 슬레이브 ID = 1, 2")
            lines.append("  4. 터미널에서 확인:")
            lines.append("     dmesg | grep ttyS")
            lines.append("     setserial /dev/ttyS1")

        self._diag_text.value = "\n".join(lines)
        self.page.update()

        self._diag_text.value = "\n".join(lines)
        self.page.update()

    @staticmethod
    def _modbus_crc(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _disconnect_motor(self):
        if self.motor_ctrl:
            self.motor_ctrl.stop_all(immediate=True)
            self.motor_ctrl.disconnect()
            self.motor_ctrl = None
        self._save_last_position()
        self._set_conn_status("Disconnected", "red")
        self.page.update()

    def _set_home(self):
        """현재 위치를 홈(0)으로 설정: 드라이버 위치 카운터 클리어 + 앱 홈 저장"""
        if self.motor_ctrl and self.motor_ctrl.connected:
            try:
                ok = self.motor_ctrl.clear_position_counter_all()
                if not ok:
                    print("[SetHome] clear_position_counter_all failed")
            except Exception as ex:
                print(f"[SetHome] error: {ex}")
            if self.motor_ctrl:
                time.sleep(0.1)
        dist_upper = self.cur_z['upper'] - self.home_z.get('upper', 0.0)
        dist_lower = self.cur_z['lower'] - self.home_z.get('lower', 0.0)
        self.stage_gap = max(0.1, self.stage_gap + dist_upper - dist_lower)
        self.home_z['upper'] = self.cur_z['upper']
        self.home_z['lower'] = self.cur_z['lower']
        self.home_angle['upper'] = self.cur_angle['upper']
        self.home_angle['lower'] = self.cur_angle['lower']

        self._save_home()
        self._update_all_graphs()
        if hasattr(self, '_status_text') and self._status_text:
            self._status_text.value = f"Home saved (Gap: {self.stage_gap:.1f}mm)"
            self._status_text.color = "#607D8B"
        self.page.update()

    def _go_home(self):
        """홈 위치로 복귀 (P1 상대좌표 이동 - 물리센서 없이 소프트웨어 홈 기준)"""
        if self.schedule_running or self._homing:
            return
        if hasattr(self, '_status_text') and self._status_text:
            self._status_text.value = "Homing..."
            self._status_text.color = "#FF9800"
        self.page.update()
        self.page.run_task(self._homing_loop)

    async def _homing_loop(self):
        """저장된 홈 기준: 홈-현재 차이(펄스)만큼 연속운전으로 복귀"""
        self._homing = True
        homing_speed = 1000

        if self.motor_ctrl and self.motor_ctrl.connected:
            self.motor_ctrl.stop_all(immediate=True)
            await asyncio.sleep(0.3)

        diff_z = {k: self.home_z[k] - self.cur_z[k] for k in ('upper', 'lower')}
        diff_angle = {k: self.home_angle[k] - self.cur_angle[k] for k in ('upper', 'lower')}

        # (motor_id, direction, duration_sec) - _map_motor_direction 적용
        tasks = []
        if abs(diff_z['upper']) > 0.01:
            dir_u = 'minus' if diff_z['upper'] > 0 else 'plus'
            dur = abs(mm_to_pulse(diff_z['upper'])) / homing_speed
            tasks.append(('upper_stage', dir_u, min(dur, 60.0)))
        if abs(diff_z['lower']) > 0.01:
            dir_l = 'plus' if diff_z['lower'] > 0 else 'minus'
            dur = abs(mm_to_pulse(diff_z['lower'])) / homing_speed
            tasks.append(('lower_stage', dir_l, min(dur, 60.0)))
        if abs(diff_angle['upper']) > 0.01:
            dir_u = 'cw' if diff_angle['upper'] > 0 else 'ccw'
            dur = abs(degree_to_pulse(diff_angle['upper'])) / homing_speed
            tasks.append(('upper_rotate', dir_u, min(dur, 60.0)))
        if abs(diff_angle['lower']) > 0.01:
            dir_l = 'cw' if diff_angle['lower'] > 0 else 'ccw'
            dur = abs(degree_to_pulse(diff_angle['lower'])) / homing_speed
            tasks.append(('lower_rotate', dir_l, min(dur, 60.0)))

        async def run_one(motor_id, direction, duration):
            if not self.motor_ctrl or not self.motor_ctrl.connected or not self._homing:
                return
            mapped = self._map_motor_direction(motor_id, direction)
            self.motor_ctrl.start_motor(motor_id, mapped, homing_speed)
            await asyncio.sleep(duration)
            if self._homing and self.motor_ctrl:
                self.motor_ctrl.stop_motor(motor_id, immediate=True)

        if tasks and self.motor_ctrl and self.motor_ctrl.connected:
            await asyncio.gather(*[run_one(m, d, t) for m, d, t in tasks])

        self.cur_z['upper'] = self.home_z['upper']
        self.cur_z['lower'] = self.home_z['lower']
        self.cur_angle['upper'] = self.home_angle['upper']
        self.cur_angle['lower'] = self.home_angle['lower']

        self._homing = False
        self._update_all_graphs()
        if hasattr(self, '_status_text') and self._status_text:
            self._status_text.value = "Returned to home" if tasks else "Already at home"
            self._status_text.color = "#4CAF50"
        self.page.update()

    def _confirm_emergency_stop(self):
        def _do_stop(e):
            self._estop_dialog.open = False
            self.page.update()
            self._emergency_stop()

        def _cancel(e):
            self._estop_dialog.open = False
            self.page.update()

        self._estop_dialog = ft.AlertDialog(
            modal=True,
            title=ft.Text("Emergency Stop", color="#ff0000", weight=ft.FontWeight.BOLD),
            content=ft.Text("모든 모터를 즉시 정지합니다.\n계속하시겠습니까?", size=14),
            actions=[
                ft.TextButton("Cancel", on_click=_cancel),
                ft.ElevatedButton("STOP", bgcolor="#ff0000", color="white", on_click=_do_stop),
            ],
            actions_alignment=ft.MainAxisAlignment.END,
        )
        self.page.overlay.append(self._estop_dialog)
        self._estop_dialog.open = True
        self.page.update()

    def _emergency_stop(self):
        if self.motor_ctrl:
            self.motor_ctrl.stop_all(immediate=True)
        self._homing = False
        self.schedule_running = False
        self._limit_alarm_triggered = False  # 알람 플래그 해제 → 이후 한계초과 재감지 가능
        for i in range(4):
            self.motor_running[i] = False
            self._highlight_motor(i, False)
        if hasattr(self, 'start_btn') and self.start_btn:
            self.start_btn.text = "Start"
        if hasattr(self, '_status_text') and self._status_text:
            self._status_text.value = "Emergency stopped"
            self._status_text.color = "#ff0000"
        self.page.update()

    # ──────────────────────────────────────────────
    # Schedule panel
    # ──────────────────────────────────────────────

    def _build_schedule_panel(self):
        self.schedule_content = ft.Column(scroll=ft.ScrollMode.AUTO, expand=True)
        self._render_schedule()
        return ft.Container(content=self.schedule_content, expand=True, padding=10)

    def _set_motor_mode(self, mode):
        if mode == "manual" and self.schedule_running:
            return
        self.motor_mode = mode
        self._render_schedule()
        self.page.update()

    def _select_motor(self, idx):
        self.selected_motor = idx
        self._render_schedule()
        self.page.update()

    def _toggle_step(self, motor_idx, step_idx):
        self.step_enabled[motor_idx][step_idx] = not self.step_enabled[motor_idx][step_idx]
        self._render_schedule()
        self.page.update()

    MAX_PPS = 8000

    def _speed_to_pps(self, val: float) -> int:
        """사용자 입력 속도(mm/s, cm/s, m/s) → pps 변환 (최대 8000pps)"""
        if self.speed_unit == "cm/s":
            mm_per_sec = val * 10.0
        elif self.speed_unit == "m/s":
            mm_per_sec = val * 1000.0
        else:
            mm_per_sec = val
        return min(self.MAX_PPS, max(0, int(mm_per_sec * PULSE_PER_MM)))

    def _on_step_change(self, motor_idx, step_idx):
        self._update_distance(motor_idx, step_idx)
        self.page.update()

    def _update_distance(self, motor_idx, step_idx):
        slot = self.steps[motor_idx][step_idx]
        if not slot['speed'] or not slot['dur'] or not slot['dist_text']:
            return
        try:
            raw = float(slot['speed'].value)
        except (ValueError, TypeError):
            raw = 0
        dur_h = self._parse_duration_to_hours(slot['dur'].value)
        mt = MOTOR_TYPES[motor_idx]
        if raw > 0 and dur_h > 0:
            if mt == 'linear':
                pps = self._speed_to_pps(raw)
                clamped = pps >= self.MAX_PPS
                dist_mm = speed_pps_to_mm_per_sec(pps) * dur_h * 3600
                warn = "⚠" if clamped else ""
                if self.speed_unit == "m/s":
                    slot['dist_text'].value = f"{warn}{dist_mm / 1000:.3f} m"
                elif self.speed_unit == "cm/s":
                    slot['dist_text'].value = f"{warn}{dist_mm / 10:.2f} cm"
                else:
                    slot['dist_text'].value = f"{warn}{dist_mm:.1f} mm"
            else:
                # raw = RPM (스테이지 기준)
                pps = min(self.MAX_PPS, max(0, int(raw * 360.0 / 60.0 / STEP_ANGLE)))
                clamped = pps >= self.MAX_PPS
                minutes = dur_h * 60.0
                revolutions = raw * minutes
                warn = "⚠" if clamped else ""
                slot['dist_text'].value = f"{warn}{revolutions:.2f} rev"
        else:
            slot['dist_text'].value = "-"

    def _cycle_speed_unit(self):
        order = ["mm/s", "cm/s", "m/s"]
        idx = order.index(self.speed_unit)
        self.speed_unit = order[(idx + 1) % 3]
        self._render_schedule()
        self.page.update()

    def _make_dir_toggle(self, motor_idx, step_idx):
        """방향 아이콘 토글 버튼"""
        mt = MOTOR_TYPES[motor_idx]
        cur = self.steps[motor_idx][step_idx].get('dir_dd', '+')
        if mt == 'linear':
            icon = ft.Icons.ARROW_UPWARD if cur == '+' else ft.Icons.ARROW_DOWNWARD
            color = "#2196F3" if cur == '+' else "#FF5722"
        else:
            icon = ft.Icons.ROTATE_RIGHT if cur == 'CW' else ft.Icons.ROTATE_LEFT
            color = "#2196F3" if cur == 'CW' else "#FF5722"
        return ft.Container(
            content=ft.Icon(icon, color="#ffffff", size=20),
            width=36, height=30, bgcolor=color, border_radius=4,
            alignment=ft.Alignment(0, 0),
            on_click=lambda e, mi=motor_idx, si=step_idx: self._toggle_dir(mi, si),
        )

    def _toggle_dir(self, motor_idx, step_idx):
        mt = MOTOR_TYPES[motor_idx]
        cur = self.steps[motor_idx][step_idx].get('dir_dd', '+')
        if mt == 'linear':
            self.steps[motor_idx][step_idx]['dir_dd'] = '-' if cur == '+' else '+'
        else:
            self.steps[motor_idx][step_idx]['dir_dd'] = 'CCW' if cur == 'CW' else 'CW'
        self._render_schedule()
        self.page.update()

    def _on_speed_dropdown_change(self, e):
        selected = e.control.value
        for i, s in enumerate(self.manual_speeds):
            if s["label"] == selected:
                self.speed_mode_idx = i
                break
        new_pps = self.manual_speeds[self.speed_mode_idx]["pps"]
        print(f"[Speed] dropdown → {selected} = {new_pps} PPS")
        if self.motor_ctrl and self.motor_ctrl.connected:
            try:
                ok = self.motor_ctrl.set_speed_all(new_pps)
                print(f"[Speed] set_speed_all({new_pps}) → {'OK' if ok else 'FAIL'}")
                if ok:
                    self._last_applied_manual_speed_pps = new_pps
            except Exception as ex:
                print(f"[Speed] error: {ex}")
        self.page.update()

    @staticmethod
    def _map_motor_direction(motor_id: str, direction: str) -> str:
        """모터별 방향 보정 (상부 스테이지는 물리 배선 기준 반전)."""
        if motor_id == "upper_stage":
            if direction == "+":
                direction = "-"
            elif direction == "-":
                direction = "+"
        return DIRECTION_MAP.get(direction, 'plus')

    def _manual_motor_start(self, motor_idx, direction):
        if self.schedule_running:
            return
        if self._would_exceed_limit(motor_idx, direction):
            return
        self._manual_pressed_at[motor_idx] = time.monotonic()
        self.motor_manual_dir[motor_idx] = direction
        if self.motor_ctrl and self.motor_ctrl.connected:
            speed = self.manual_speeds[self.speed_mode_idx]["pps"]
            motor_id = MOTOR_IDS[motor_idx]
            mapped_dir = self._map_motor_direction(motor_id, direction)
            print(f"[Manual] START motor={motor_id} dir={mapped_dir} speed={speed}PPS")
            try:
                ok = self.motor_ctrl.start_motor(motor_id, mapped_dir, speed)
                print(f"[Manual] start → {'OK' if ok else 'FAIL'}")
                if ok:
                    self.motor_running[motor_idx] = True
                    self._highlight_motor(motor_idx, True)
            except Exception as ex:
                print(f"Manual start error: {ex}")
        else:
            # Keep simulator-like behavior while disconnected.
            self.motor_running[motor_idx] = True
            self._highlight_motor(motor_idx, True)

    async def _delayed_manual_stop(self, motor_idx: int, delay_sec: float):
        await asyncio.sleep(max(0.0, delay_sec))
        self._do_manual_stop(motor_idx)

    def _do_manual_stop(self, motor_idx):
        self.motor_running[motor_idx] = False
        self._highlight_motor(motor_idx, False)
        if self.motor_ctrl and self.motor_ctrl.connected:
            motor_id = MOTOR_IDS[motor_idx]
            try:
                self.motor_ctrl.stop_motor(motor_id)
            except Exception as ex:
                print(f"Manual stop error: {ex}")

    def _manual_motor_stop(self, motor_idx):
        if self.schedule_running:
            return
        elapsed = time.monotonic() - self._manual_pressed_at[motor_idx]
        # 회전축은 짧은 탭에서 체감이 어려워 최소 구동시간을 더 길게 준다.
        min_run = 0.4 if MOTOR_TYPES[motor_idx] == 'rotate' else 0.15
        if elapsed < min_run:
            asyncio.create_task(self._delayed_manual_stop(motor_idx, min_run - elapsed))
            return
        self._do_manual_stop(motor_idx)

    def _highlight_motor(self, motor_idx, active):
        lbl = self._motor_labels[motor_idx]
        row_c = self._motor_rows[motor_idx]
        if lbl:
            lbl.color = "#00E676" if active else "#333333"
            lbl.value = f"● {MOTOR_LABELS[motor_idx]}" if active else MOTOR_LABELS[motor_idx]
        if row_c:
            row_c.bgcolor = "rgba(0,230,118,0.12)" if active else "transparent"
        try:
            self.page.update()
        except Exception:
            pass

    @staticmethod
    def _jog_icon(direction: str):
        icon_map = {
            '+': ft.Icons.ARROW_UPWARD,
            '-': ft.Icons.ARROW_DOWNWARD,
            'CW': ft.Icons.ROTATE_RIGHT,
            'CCW': ft.Icons.ROTATE_LEFT,
        }
        return icon_map.get(direction, ft.Icons.PLAY_ARROW)

    def _make_jog_button(self, label, motor_idx, direction, width=100, height=60):
        """누르고 있는 동안 모터 작동, 떼면 정지"""
        icon = self._jog_icon(direction)
        return ft.GestureDetector(
            content=ft.Container(
                content=ft.Column([
                    ft.Icon(icon, color="#ffffff", size=24),
                    ft.Text(label, size=10, color="#ffffff", text_align=ft.TextAlign.CENTER),
                ], spacing=2, horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                width=width, height=height, bgcolor="#003366", border_radius=8,
                alignment=ft.Alignment(0, 0),
                border=ft.Border.all(2, "#001a33"),
            ),
            on_tap_down=lambda e, mi=motor_idx, d=direction: self._manual_motor_start(mi, d),
            on_tap_up=lambda e, mi=motor_idx: self._manual_motor_stop(mi),
        )

    def _render_schedule(self):
        self.schedule_content.controls.clear()

        mode_row = ft.Row(alignment=ft.MainAxisAlignment.CENTER, spacing=8)
        for mode, label in [("schedule", "Schedule"), ("manual", "Manual")]:
            bg = "#003366" if self.motor_mode == mode else "#cccccc"
            fg = "#ffffff" if self.motor_mode == mode else "#333333"
            mode_row.controls.append(ft.Container(
                content=ft.Text(label, size=13, weight=ft.FontWeight.BOLD,
                                color=fg, text_align=ft.TextAlign.CENTER),
                width=100, height=36, bgcolor=bg, border_radius=6,
                alignment=ft.Alignment(0, 0),
                on_click=lambda e, m=mode: self._set_motor_mode(m),
            ))
        self.schedule_content.controls.append(mode_row)
        self.schedule_content.controls.append(ft.Divider(height=1))

        if self.motor_mode == "schedule":
            self._render_schedule_mode()
        else:
            self._render_manual_mode()

    def _render_schedule_mode(self):
        C_W, C_H = 80, 34

        motor_btns = ft.Row(alignment=ft.MainAxisAlignment.CENTER, spacing=6)
        for i in range(4):
            bg = "#003366" if i == self.selected_motor else "#cccccc"
            fg = "#ffffff" if i == self.selected_motor else "#333333"
            motor_btns.controls.append(ft.Container(
                content=ft.Text(MOTOR_SHORT[i], size=11, weight=ft.FontWeight.BOLD,
                                color=fg, text_align=ft.TextAlign.CENTER),
                width=90, height=34, bgcolor=bg, border_radius=4,
                alignment=ft.Alignment(0, 0),
                on_click=lambda e, idx=i: self._select_motor(idx),
            ))
        self.schedule_content.controls.append(motor_btns)
        self.schedule_content.controls.append(ft.Container(height=6))

        m = self.selected_motor
        mt = MOTOR_TYPES[m]
        dir_opts = DIRECTION_OPTIONS[mt]

        if mt == 'linear':
            speed_header = f"Speed ({self.speed_unit})"
        else:
            speed_header = "Speed (rpm)"
        dist_header = "Distance" if mt == 'linear' else "Rotation"

        unit_btn = ft.Container(
            content=ft.Text(self.speed_unit, size=11, weight=ft.FontWeight.BOLD,
                            color="#ffffff", text_align=ft.TextAlign.CENTER),
            width=55, height=28, bgcolor="#607D8B", border_radius=4,
            alignment=ft.Alignment(0, 0),
            on_click=lambda e: self._cycle_speed_unit(),
        )
        self.schedule_content.controls.append(
            ft.Row([ft.Text("Unit:", size=11), unit_btn], alignment=ft.MainAxisAlignment.CENTER, spacing=6))
        self.schedule_content.controls.append(ft.Container(height=4))

        for step in range(8):
            slot = self.steps[m][step]
            if not slot['speed']:
                default_speed = "10" if mt == 'linear' else "90"
                slot['speed'] = self._make_table_input(
                    default_speed, w=C_W, h=C_H,
                    on_change=lambda e, mi=m, si=step: self._on_step_change(mi, si))
                slot['dur'] = self._make_duration_input(
                    "00:00:30", w=C_W, h=C_H,
                    on_change=lambda e, mi=m, si=step: self._on_step_change(mi, si))
                slot['dir_dd'] = dir_opts[0]
                slot['dist_text'] = ft.Text("-", size=10, text_align=ft.TextAlign.CENTER)
            self._update_distance(m, step)

        data_rows = []
        for step in range(8):
            slot = self.steps[m][step]
            toggle = self._make_step_toggle(
                f"S{step + 1}", self.step_enabled[m][step],
                lambda e, mi=m, si=step: self._toggle_step(mi, si))
            dir_btn = self._make_dir_toggle(m, step)
            data_rows.append([
                toggle, slot['speed'], slot['dur'], dir_btn, slot['dist_text'],
            ])

        table = self._build_table(
            headers=["Step", speed_header, "Duration", "Dir", dist_header],
            data_rows=data_rows,
            col_widths=[50, C_W, C_W, 50, 90],
            row_height=C_H,
        )
        self.schedule_content.controls.append(
            ft.Row([table], alignment=ft.MainAxisAlignment.CENTER))

        self.schedule_content.controls.append(ft.Container(height=10))

        btn_row = ft.Row(alignment=ft.MainAxisAlignment.CENTER, spacing=10)
        self.apply_btn = ft.ElevatedButton("Apply", bgcolor="#2196F3", color="white",
                                           on_click=lambda e: self._apply_schedule())
        self.start_btn = ft.ElevatedButton("Start", bgcolor="#4CAF50", color="white",
                                           on_click=lambda e: self._start_schedule())
        btn_row.controls.extend([self.apply_btn, self.start_btn])
        self.schedule_content.controls.append(btn_row)

        self._status_text = ft.Text("", size=12)
        self.schedule_content.controls.append(self._status_text)

        self.schedule_content.controls.append(ft.Container(height=30))
        estop_btn = ft.Container(
            content=ft.Text("EMERGENCY STOP", size=14, weight=ft.FontWeight.BOLD,
                            color="#ffffff", text_align=ft.TextAlign.CENTER),
            width=200, height=44, bgcolor="#ff0000", border_radius=8,
            alignment=ft.Alignment(0, 0),
            on_click=lambda e: self._confirm_emergency_stop(),
        )
        self.schedule_content.controls.append(
            ft.Row([estop_btn], alignment=ft.MainAxisAlignment.CENTER))

    def _render_manual_mode(self):
        self._speed_dropdown = ft.Dropdown(
            width=200, value=self.manual_speeds[self.speed_mode_idx]["label"],
            text_size=14, dense=True,
            options=[ft.DropdownOption(s["label"]) for s in self.manual_speeds],
            on_select=lambda e: self._on_speed_dropdown_change(e),
        )
        self.schedule_content.controls.append(
            ft.Row([ft.Text("Speed:", size=14, weight=ft.FontWeight.BOLD),
                    self._speed_dropdown],
                   alignment=ft.MainAxisAlignment.CENTER))
        self.schedule_content.controls.append(ft.Container(height=10))

        for mi in range(4):
            mt = MOTOR_TYPES[mi]
            if mt == 'linear':
                dirs = [('+', '+'), ('-', '-')]
            else:
                dirs = [('CW', 'CW'), ('CCW', 'CCW')]

            is_active = self.motor_running[mi]
            lbl_text = f"● {MOTOR_LABELS[mi]}" if is_active else MOTOR_LABELS[mi]
            lbl_color = "#00E676" if is_active else "#333333"
            label = ft.Text(lbl_text, size=13, weight=ft.FontWeight.BOLD,
                            width=110, text_align=ft.TextAlign.RIGHT, color=lbl_color)
            self._motor_labels[mi] = label
            btns = [self._make_jog_button(d[0], mi, d[1], width=90, height=50) for d in dirs]

            row_bg = "rgba(0,230,118,0.12)" if is_active else "transparent"
            row_container = ft.Container(
                content=ft.Row(
                    [label, ft.Container(width=10)] + btns,
                    alignment=ft.MainAxisAlignment.CENTER, spacing=8,
                ),
                bgcolor=row_bg, border_radius=6, padding=4,
            )
            self._motor_rows[mi] = row_container
            self.schedule_content.controls.append(row_container)
            self.schedule_content.controls.append(ft.Container(height=6))

        self.schedule_content.controls.append(ft.Divider())

        home_set_btn = ft.ElevatedButton("Set Home", bgcolor="#607D8B", color="white",
                                         on_click=lambda e: self._set_home())
        home_go_btn = ft.ElevatedButton("Go Home", bgcolor="#FF9800", color="white",
                                        on_click=lambda e: self._go_home())
        self.schedule_content.controls.append(
            ft.Row([home_set_btn, home_go_btn], alignment=ft.MainAxisAlignment.CENTER, spacing=10))
        self.schedule_content.controls.append(ft.Container(height=6))

        stop_btn = ft.ElevatedButton("STOP ALL", bgcolor="#ff0000", color="white",
                                     on_click=lambda e: self._confirm_emergency_stop())
        self.schedule_content.controls.append(
            ft.Row([stop_btn], alignment=ft.MainAxisAlignment.CENTER))

        self._status_text = ft.Text("", size=12)
        self.schedule_content.controls.append(self._status_text)

    # ──────────────────────────────────────────────
    # Graph
    # ──────────────────────────────────────────────

    def _get_schedule_segments(self, motor_idx):
        """각 스텝의 (시작시간, 종료시간, 속도pps, 방향) 리스트 반환 (시간 단위: hours)"""
        segments = []
        cum = 0.0
        for i, slot in enumerate(self.steps[motor_idx]):
            if not self.step_enabled[motor_idx][i]:
                continue
            df = slot.get('dur')
            dd = slot.get('dir_dd')
            spd = self._speed_input_to_pps(motor_idx, i)
            dur = self._parse_duration_to_hours(df.value) if df and df.value else 0.0
            d = dd if isinstance(dd, str) else (dd.value if dd else '+')
            if dur > 0:
                segments.append((cum, cum + dur, spd, d))
                cum += dur
        return segments

    def _get_global_total_duration(self) -> float:
        return max(self._get_total_duration(mi) for mi in range(4))

    @staticmethod
    def _dir_arrow(direction: str) -> str:
        return {'+': '↑', '-': '↓', 'CW': '↻', 'CCW': '↺'}.get(direction, direction)

    def _seg_label(self, motor_idx, spd, dur_h, direction):
        """타임라인 바 안에 표시할 텍스트: 방향아이콘 + 거리/각도"""
        arrow = self._dir_arrow(direction)
        seconds = dur_h * 3600
        mt = MOTOR_TYPES[motor_idx]
        if mt == 'linear':
            dist_mm = speed_pps_to_mm_per_sec(int(spd)) * seconds
            if self.speed_unit == "m/s":
                return f"{arrow} {dist_mm / 1000:.3f}m"
            elif self.speed_unit == "cm/s":
                return f"{arrow} {dist_mm / 10:.2f}cm"
            else:
                return f"{arrow} {dist_mm:.1f}mm"
        else:
            deg = speed_pps_to_deg_per_sec(int(spd)) * seconds
            if deg >= 360:
                return f"{arrow} {deg / 360:.1f}rev"
            return f"{arrow} {deg:.0f}°"

    @staticmethod
    def _fig_to_b64(fig) -> str:
        buf = io.BytesIO()
        fig.savefig(buf, format='png', bbox_inches='tight')
        plt.close(fig)
        buf.seek(0)
        return f"data:image/png;base64,{base64.b64encode(buf.read()).decode('ascii')}"

    def _render_stage_diagram(self) -> str:
        """통합 스테이지 도식: 홈 대비 이동거리 표시"""
        if not plt:
            return ""
        fig = plt.figure(figsize=(3.2, 4.5), dpi=100)
        ax = fig.add_subplot(111)

        dist_upper = self.cur_z['upper'] - self.home_z['upper']
        dist_lower = self.cur_z['lower'] - self.home_z['lower']

        lower_pos = 0.0 + dist_lower
        upper_pos = self.stage_gap + dist_upper
        gap_now = upper_pos - lower_pos

        margin = max(self.stage_gap * 0.3, 10)
        y_top = self.stage_gap + margin
        y_bot = -margin

        ax.set_xlim(-1, 1)
        ax.set_ylim(y_bot, y_top)

        ax.fill_between([-0.6, 0.6], upper_pos, y_top, color='#E3F2FD', alpha=0.4)
        ax.fill_between([-0.6, 0.6], y_bot, lower_pos, color='#E8F5E9', alpha=0.4)

        ax.axhline(y=upper_pos, color='#1565C0', linewidth=3.5)
        ax.plot(0, upper_pos, 'v', color='#1565C0', markersize=12, zorder=5)
        ax.text(0.65, upper_pos, f"Upper\n{dist_upper:+.1f} mm",
                va='center', fontsize=9, fontweight='bold', color='#1565C0')

        ax.axhline(y=lower_pos, color='#2E7D32', linewidth=3.5)
        ax.plot(0, lower_pos, '^', color='#2E7D32', markersize=12, zorder=5)
        ax.text(0.65, lower_pos, f"Lower\n{dist_lower:+.1f} mm",
                va='center', fontsize=9, fontweight='bold', color='#2E7D32')

        mid = (upper_pos + lower_pos) / 2
        ax.annotate('', xy=(0.45, upper_pos), xytext=(0.45, lower_pos),
                    arrowprops=dict(arrowstyle='<->', color='#FF5722', lw=1.5))
        ax.text(-0.85, mid, f"Gap\n{gap_now:.1f}mm",
                ha='center', va='center', fontsize=10, fontweight='bold', color='#FF5722',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#FFF3E0', edgecolor='#FF5722', alpha=0.9))

        ax.axhline(y=0, color='gray', linewidth=0.8, linestyle='--', alpha=0.5)
        ax.axhline(y=self.stage_gap, color='gray', linewidth=0.8, linestyle='--', alpha=0.5)

        ax.set_xticks([])
        ax.set_ylabel("Position (mm)", fontsize=10)
        ax.tick_params(axis='y', labelsize=9)
        ax.set_title("Stage Position", fontsize=12, fontweight='bold')
        ax.grid(axis='y', alpha=0.2)
        for spine in ['top', 'right', 'bottom']:
            ax.spines[spine].set_visible(False)

        fig.tight_layout()
        return self._fig_to_b64(fig)

    def _render_gantt(self) -> str:
        """스케줄 간트 차트 (타임라인)"""
        if not plt:
            return ""
        fig = plt.figure(figsize=(3.8, 2.4), dpi=100)
        ax = fig.add_subplot(111)

        global_h = self._get_global_total_duration()
        if global_h <= 0:
            global_h = 0.01
        total_s = max(int(global_h * 3600), 10)

        if total_s < 60:
            t_unit, t_div = "s", 1.0
        elif total_s < 3600:
            t_unit, t_div = "min", 60.0
        else:
            t_unit, t_div = "h", 3600.0

        bar_colors = ['#42A5F5', '#66BB6A', '#FFA726', '#AB47BC']
        gantt_labels = ['U-Stage', 'L-Stage', 'U-Rotate', 'L-Rotate']
        gantt_order = [0, 2, 1, 3]

        for row, mi in enumerate(gantt_order):
            segs = self._get_schedule_segments(mi)
            for (t0, t1, spd, d) in segs:
                if spd > 0:
                    x0 = t0 * 3600 / t_div
                    w = (t1 - t0) * 3600 / t_div
                    ax.barh(row, w, left=x0, height=0.6,
                            color=bar_colors[row], alpha=0.85,
                            edgecolor='#333333', linewidth=0.5)
                    mid = x0 + w / 2
                    label = self._seg_label(mi, spd, t1 - t0, d)
                    fontsize = 6 if len(label) > 10 else 7
                    ax.text(mid, row, label, ha='center', va='center',
                            fontsize=fontsize, color='white', fontweight='bold')

        if self.schedule_running and self.history[0]:
            elapsed_disp = len(self.history[0]) / t_div
            ax.axvline(x=elapsed_disp, color='red', linewidth=1.5, linestyle='-', alpha=0.8)

        ax.set_yticks(range(4))
        ax.set_yticklabels(gantt_labels, fontsize=9)
        ax.set_xlim(0, total_s / t_div)
        ax.xaxis.tick_top()
        ax.tick_params(axis='x', labelsize=8)
        ax.invert_yaxis()
        ax.grid(axis='x', alpha=0.3)
        ax.set_axisbelow(True)
        ax.set_title(f"Timeline ({t_unit})", fontsize=11, fontweight='bold', pad=16)
        fig.tight_layout()
        return self._fig_to_b64(fig)

    def _update_abs_position(self):
        """현재 스케줄/수동 상태로부터 절대좌표 갱신 (100ms마다 호출)"""
        if self._schedule_relative_mode and self.schedule_running:
            return  # 상대위치 스케줄: 위치는 move_relative 시점에 반영됨
        dt = self._position_dt
        for mi, key in [(0, 'upper'), (2, 'lower')]:
            if self.motor_running[mi] or (self.schedule_running and self.history[mi]):
                spd_pps = 0
                d = '+'
                if self.schedule_running and self.history[mi]:
                    spd_pps = self.history[mi][-1] if self.history[mi] else 0
                    d = self._get_direction_at(mi, self.schedule_time)
                elif self.motor_running[mi]:
                    spd_pps = self.manual_speeds[self.speed_mode_idx]["pps"]
                    d = self.motor_manual_dir[mi]
                mm_per_sec = spd_pps / PULSE_PER_MM
                sign = -1 if d in ('-', 'minus', 'down') else 1
                self.cur_z[key] += sign * mm_per_sec * dt

        ok, msg = self._check_stage_limits()
        if not ok:
            self._trigger_limit_alarm(msg)

        for mi, key in [(1, 'upper'), (3, 'lower')]:
            if self.motor_running[mi] or (self.schedule_running and self.history[mi]):
                spd_pps = 0
                d = 'CW'
                if self.schedule_running and self.history[mi]:
                    spd_pps = self.history[mi][-1] if self.history[mi] else 0
                    d = self._get_direction_at(mi, self.schedule_time)
                elif self.motor_running[mi]:
                    spd_pps = self.manual_speeds[self.speed_mode_idx]["pps"]
                    d = self.motor_manual_dir[mi]
                deg_per_sec = spd_pps * STEP_ANGLE
                sign = -1 if d in ('CCW', 'ccw') else 1
                self.cur_angle[key] += sign * deg_per_sec * dt

    def _update_all_graphs(self):
        """간트 차트 + 스테이지 다이어그램 갱신"""
        try:
            self.gantt_img.src = self._render_gantt()
            self.gantt_img.update()
            self.stage_diagram_img.src = self._render_stage_diagram()
            self.stage_diagram_img.update()
        except Exception:
            pass

    async def _graph_loop(self):
        """위치 계산은 100ms, 그래프 렌더링은 1초 주기"""
        self._graph_loop_running = True
        elapsed_since_render = 0.0
        _last_tick = time.monotonic()
        while self._graph_loop_running:
            try:
                _now = time.monotonic()
                self._position_dt = min(_now - _last_tick, 0.5)  # 실제 경과시간 사용, 최대 0.5s 캡
                _last_tick = _now
                self._update_abs_position()
                elapsed_since_render += self._position_dt
                if elapsed_since_render >= self._graph_render_interval:
                    self._update_all_graphs()
                    elapsed_since_render = 0.0
            except Exception as ex:
                print(f"Graph loop error: {ex}")
            await asyncio.sleep(0.1)

    # ──────────────────────────────────────────────
    # Schedule logic
    # ──────────────────────────────────────────────

    def _get_total_duration(self, motor_idx) -> float:
        total = 0.0
        for i, slot in enumerate(self.steps[motor_idx]):
            if not self.step_enabled[motor_idx][i]:
                continue
            df = slot.get('dur')
            total += self._parse_duration_to_hours(df.value) if df and df.value else 0.0
        return total

    def _speed_input_to_pps(self, motor_idx, step_idx) -> float:
        slot = self.steps[motor_idx][step_idx]
        sp = slot.get('speed')
        try:
            raw = float(sp.value) if sp and sp.value else 0
        except (ValueError, TypeError):
            raw = 0
        if raw <= 0:
            return 0
        mt = MOTOR_TYPES[motor_idx]
        if mt == 'linear':
            return self._speed_to_pps(raw)
        else:
            # RPM → PPS: 1 RPM = 360°/60s / (STEP_ANGLE °/pulse)
            return min(self.MAX_PPS, max(0, int(raw * 360.0 / 60.0 / STEP_ANGLE)))

    def _get_speed_at(self, motor_idx, t_hours) -> float:
        steps_list = []
        for i, slot in enumerate(self.steps[motor_idx]):
            if not self.step_enabled[motor_idx][i]:
                continue
            spd = self._speed_input_to_pps(motor_idx, i)
            df = slot.get('dur')
            dur = self._parse_duration_to_hours(df.value) if df and df.value else 0.0
            steps_list.append((spd, dur))

        if not steps_list:
            return 0

        cum = 0.0
        for spd, dur in steps_list:
            if dur <= 0:
                continue
            if t_hours < cum + dur:
                # Step schedule should be piecewise-constant, not ramp-interpolated.
                return spd
            cum += dur
        # After the final step, speed is zero.
        return 0

    def _get_direction_at(self, motor_idx, t_hours) -> str:
        steps_list = []
        for i, slot in enumerate(self.steps[motor_idx]):
            if not self.step_enabled[motor_idx][i]:
                continue
            df = slot.get('dur')
            dd = slot.get('dir_dd')
            dur = self._parse_duration_to_hours(df.value) if df and df.value else 0.0
            d = dd if isinstance(dd, str) else (dd.value if dd else '+')
            steps_list.append((d, dur))

        if not steps_list:
            return '+'

        cum = 0.0
        cur_dir = steps_list[0][0]
        for d, dur in steps_list:
            cur_dir = d
            if t_hours < cum + dur:
                return cur_dir
            cum += dur
        return cur_dir

    # 스케줄 모드 펄스 배율 (실측 보정)
    # 선형: ×2→20mm, 목표25mm → 2×(25/20)=2.5
    # 회전: ×6→70°, 목표50° → 6×(50/70)≈4.3
    SCHEDULE_PULSE_MULT = {'linear': 2.5, 'rotate': 1.0}

    def _get_relative_segments(self, motor_idx) -> list:
        """상대위치용: (delta_pulse, speed_pps, duration_sec, direction) 리스트"""
        segs = []
        mt = MOTOR_TYPES[motor_idx]
        pulse_mult = self.SCHEDULE_PULSE_MULT.get(mt, 1)
        for i, slot in enumerate(self.steps[motor_idx]):
            if not self.step_enabled[motor_idx][i]:
                continue
            spd = int(self._speed_input_to_pps(motor_idx, i))
            df = slot.get('dur')
            dd = slot.get('dir_dd')
            dur_h = self._parse_duration_to_hours(df.value) if df and df.value else 0.0
            dur_sec = dur_h * 3600.0
            d = dd if isinstance(dd, str) else (dd.value if dd else '+')
            if spd <= 0 or dur_sec <= 0:
                continue
            # 스케줄 펄스 배율 적용 (선형 ×2, 회전 ×4.3)
            spd_actual = min(self.MAX_PPS, int(spd * pulse_mult))
            sign = 1 if d in ('+', 'plus', 'CW', 'cw') else -1
            if MOTOR_IDS[motor_idx] == 'upper_stage':
                sign *= -1  # 상부 스테이지 방향 반전
            delta_pulse = sign * int(spd_actual * dur_sec)
            segs.append((delta_pulse, spd_actual, dur_sec, d))
        return segs

    def _apply_schedule(self):
        self.history = [[] for _ in range(4)]
        self.schedule_time = 0.0
        self.schedule_running = False
        self._update_all_graphs()
        self._status_text.value = "Schedule preview applied"
        self._status_text.color = "#2196F3"
        self.page.update()

    def _start_schedule(self):
        if self.schedule_running:
            self.schedule_running = False
            if self.motor_ctrl:
                self.motor_ctrl.stop_all()
            self.start_btn.text = "Start"
            self._status_text.value = "Stopped"
            self.page.update()
            return

        self.history = [[] for _ in range(4)]
        self.schedule_time = 0.0
        self._last_schedule_cmd = [{"spd": None, "dir": None} for _ in range(4)]
        self.schedule_running = True
        self.start_btn.text = "Stop"
        self._status_text.value = "Running..."
        self._status_text.color = "#4CAF50"
        self.page.update()
        self.page.run_task(self._schedule_loop)

    async def _schedule_loop(self):
        """연속 이동 방식: start_motor → duration 대기 → stop_motor"""
        total_dur = max(self._get_total_duration(mi) for mi in range(4))
        if total_dur <= 0:
            self.schedule_running = False
            self.start_btn.text = "Start"
            self._status_text.value = "No schedule"
            self.page.update()
            return

        total_sec = total_dur * 3600.0 + 1.0
        start_time = time.monotonic()

        async def run_motor_segments(motor_idx: int):
            """연속운전: 방향·속도·시간으로 세그먼트 실행"""
            motor_id = MOTOR_IDS[motor_idx]
            segs = self._get_relative_segments(motor_idx)
            for j, (delta_pulse, spd, dur_sec, d) in enumerate(segs):
                if not self.schedule_running:
                    if self.motor_ctrl:
                        self.motor_ctrl.stop_motor(motor_id, immediate=False)  # P0 0501/0502
                    return
                # 경과시간 기반 강제 정지: 계획시간 초과 시 세그먼트 시작하지 않음
                if time.monotonic() - start_time >= total_sec:
                    if self.motor_ctrl:
                        self.motor_ctrl.stop_motor(motor_id, immediate=False)  # P0 0501/0502
                    return
                if not self.motor_ctrl or not self.motor_ctrl.connected:
                    await asyncio.sleep(min(dur_sec, max(0, total_sec - (time.monotonic() - start_time))))
                    continue
                try:
                    mapped = self._map_motor_direction(motor_id, d)
                    self.motor_ctrl.start_motor(motor_id, mapped, spd)
                    limit_hit = False
                    # 위치 추적용 delta_mm: 배율 제거한 실제 물리 이동거리
                    mt = MOTOR_TYPES[motor_idx]
                    pulse_mult = self.SCHEDULE_PULSE_MULT.get(mt, 1)
                    if motor_id in ('upper_stage', 'lower_stage'):
                        start_z = self.cur_z['upper'] if motor_id == 'upper_stage' else self.cur_z['lower']
                        raw_delta = (-delta_pulse if motor_id == 'upper_stage' else delta_pulse)
                        delta_mm = raw_delta / PULSE_PER_MM / pulse_mult
                    elapsed = 0.0
                    while elapsed < dur_sec and self.schedule_running:
                        if time.monotonic() - start_time >= total_sec:
                            break
                        if motor_id in ('upper_stage', 'lower_stage'):
                            est_z = start_z + (elapsed / dur_sec) * delta_mm
                            if est_z < 0 or est_z > MAX_TRAVEL_MM:
                                limit_hit = True
                                break
                        chunk = min(0.1, dur_sec - elapsed)
                        await asyncio.sleep(chunk)
                        elapsed += chunk
                    self.motor_ctrl.stop_motor(motor_id, immediate=False)  # P0 0501/0502
                    if not self.schedule_running or time.monotonic() - start_time >= total_sec:
                        return
                    if motor_id == 'upper_stage':
                        actual_delta = (elapsed / dur_sec) * delta_mm if limit_hit else delta_mm
                        self.cur_z['upper'] += actual_delta
                    elif motor_id == 'lower_stage':
                        actual_delta = (elapsed / dur_sec) * delta_mm if limit_hit else delta_mm
                        self.cur_z['lower'] += actual_delta
                    elif motor_id == 'upper_rotate':
                        # 회전도 배율 제거한 실제 각도로 추적
                        self.cur_angle['upper'] += (delta_pulse / pulse_mult) * STEP_ANGLE
                    else:
                        self.cur_angle['lower'] += (delta_pulse / pulse_mult) * STEP_ANGLE
                    if limit_hit:
                        self.schedule_running = False
                        ok, msg = self._check_stage_limits()
                        self._trigger_limit_alarm(msg or "스테이지 한계 초과")
                        return
                    ok, msg = self._check_stage_limits()
                    if not ok:
                        self.schedule_running = False
                        self._trigger_limit_alarm(msg)
                        return
                except Exception as ex:
                    print(f"[Schedule] {motor_id} seg{j} error: {ex}")
                    await asyncio.sleep(dur_sec)

        async def history_updater():
            """그래프용 진행 표시 (모터 실행과 병렬)"""
            while self.schedule_running and self.schedule_time < total_dur:
                for mi in range(4):
                    self.history[mi].append(int(self._get_speed_at(mi, self.schedule_time)))
                self.schedule_time += self._schedule_cmd_dt / 3600.0
                await asyncio.sleep(self._schedule_cmd_dt)

        async def schedule_watchdog():
            """계획시간 도달 시 강제 정지"""
            await asyncio.sleep(total_sec)
            if self.schedule_running:
                self.schedule_running = False
                if self.motor_ctrl:
                    # P0 방식: 0x0501(X축), 0x0502(Y축) 감속 정지
                    self.motor_ctrl.stop_all(immediate=False)
                print("[Schedule] Watchdog: planned time reached, P0 0501/0502 sent")

        # 4축 병렬 + history + watchdog 동시 실행
        await asyncio.gather(
            asyncio.gather(*[run_motor_segments(mi) for mi in range(4)]),
            history_updater(),
            schedule_watchdog(),
        )

        if self.motor_ctrl:
            # P0 방식: 0x0501(X축), 0x0502(Y축) 감속 정지
            self.motor_ctrl.stop_all(immediate=False)
        self.schedule_running = False
        self.start_btn.text = "Start"
        self._status_text.value = "Schedule complete"
        self._status_text.color = "#2196F3"
        self._update_all_graphs()
        self.page.update()

    # ──────────────────────────────────────────────
    # Main layout
    # ──────────────────────────────────────────────

    def main(self, page: ft.Page):
        self.page = page
        page.title = "Smart Motor Control"
        page.window.width = 1024
        page.window.height = 600
        page.padding = 0
        page.bgcolor = "#f0f0f0"

        def on_window_event(e):
            if e.data in ("close", "destroy"):
                self._save_last_position()

        page.window.on_event = on_window_event

        self._build_numpad()
        self._build_duration_pad()

        placeholder_b64 = (
            "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAAC0lE"
            "QVQIHWNgAAIABQABNjN9GQAAAAlwSFlzAAAWJQAAFiUBSVIk8AAAAAtJREFUCB1jYAACAAAFAAGbfEHV"
            "AAAAAElFTkSuQmCC"
        )
        self.gantt_img = ft.Image(src=placeholder_b64, width=380, height=240)
        self.stage_diagram_img = ft.Image(src=placeholder_b64, width=320, height=400)

        self.current_view = "schedule"

        nav_col = ft.Column([
            self._nav_btn("Motor", lambda e: self._switch_view("schedule")),
            self._nav_btn("Settings", lambda e: self._switch_view("settings")),
        ], spacing=4, width=90)

        self.main_content = ft.Column(expand=True)

        left_panel = ft.Container(
            content=ft.Row([
                ft.Container(content=nav_col, width=90, bgcolor="#e8e8e8", padding=6),
                ft.Container(content=self.main_content, expand=True),
            ], spacing=0, expand=True),
            expand=7,
        )

        right_panel = ft.Container(
            content=ft.Column([
                self.gantt_img,
                self.stage_diagram_img,
            ], spacing=4, scroll=ft.ScrollMode.AUTO, horizontal_alignment=ft.CrossAxisAlignment.CENTER),
            expand=3, bgcolor="#ffffff", border_radius=8, padding=5,
        )

        layout = ft.Row([left_panel, right_panel], expand=True, spacing=4)

        self._switch_view("schedule")

        page.add(ft.Stack([layout, self._numpad_overlay, self._durpad_overlay], expand=True))

        page.run_task(self._graph_loop)
        page.run_task(self._auto_connect_on_start)

    def _nav_btn(self, label, on_click):
        return ft.Container(
            content=ft.Text(label, size=13, weight=ft.FontWeight.BOLD,
                            color="#ffffff", text_align=ft.TextAlign.CENTER),
            width=80, height=50, bgcolor="#003366", border_radius=6,
            alignment=ft.Alignment(0, 0), on_click=on_click,
        )

    def _switch_view(self, view):
        self.current_view = view
        self.main_content.controls.clear()
        if view == "schedule":
            self.main_content.controls.append(self._build_schedule_panel())
        else:
            self.main_content.controls.append(self._build_settings())
        self.page.update()


def main(page: ft.Page):
    app = MotorApp()
    app.main(page)


if __name__ == "__main__":
    ft.app(target=main)
