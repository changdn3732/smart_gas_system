"""
Smart Motor Control - PMC-2HSP 모터 제어 앱
4개 모터 (상부 스테이지/회전, 하부 스테이지/회전) 스케줄 제어
"""
import flet as ft
import asyncio
import math
import base64
import io
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from devices.motor_controller import (
    MotorController, PULSE_PER_MM, STEP_ANGLE,
    speed_pps_to_mm_per_sec, speed_pps_to_deg_per_sec,
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


class MotorApp:

    def __init__(self):
        self.motor_ctrl: MotorController | None = None
        self.selected_motor = 0

        self.steps = [
            [{'speed': None, 'dur': None, 'dir_dd': None, 'dist_text': None} for _ in range(8)]
            for _ in range(4)
        ]
        self.step_enabled = [
            [True, True, True, True, False, False, False, False]
            for _ in range(4)
        ]

        self.schedule_running = False
        self.schedule_time = 0.0
        self.history = [[] for _ in range(4)]

        self.motor_mode = "schedule"  # "schedule" or "manual"
        self.speed_mode = "low"       # "low" or "high"
        self.low_speed = 500
        self.high_speed = 3000
        self.motor_running = [False, False, False, False]
        self._motor_labels = [None, None, None, None]
        self._motor_rows = [None, None, None, None]

    # ──────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────

    @staticmethod
    def _parse_duration_to_hours(text: str) -> float:
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
            return float(text)
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
            deg = speed_pps_to_deg_per_sec(speed_pps) * seconds
            revs = deg / 360.0
            if revs >= 1.0:
                return f"{revs:.2f} rev"
            return f"{deg:.1f}°"

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
            width=200, value="COM7", text_size=13, dense=True,
            options=self._scan_ports(),
        )
        self._baud_input = self._make_table_input("9600", w=100, h=34)

        self._conn_status = ft.Text("Disconnected", color="red", size=13)

        refresh_btn = ft.ElevatedButton("Refresh Ports", on_click=lambda e: self._refresh_ports())
        connect_btn = ft.ElevatedButton("Connect", bgcolor="#4CAF50", color="white",
                                        on_click=lambda e: self._connect_motor())
        disconnect_btn = ft.ElevatedButton("Disconnect", bgcolor="#f44336", color="white",
                                           on_click=lambda e: self._disconnect_motor())
        stop_all_btn = ft.ElevatedButton("STOP ALL", bgcolor="#ff0000", color="white",
                                         on_click=lambda e: self._emergency_stop())

        return ft.Container(
            content=ft.Column([
                ft.Text("Motor Settings", size=16, weight=ft.FontWeight.BOLD),
                ft.Divider(),
                ft.Row([ft.Text("Port:", width=80), self._port_dropdown, refresh_btn]),
                ft.Row([ft.Text("Baudrate:", width=80), self._baud_input]),
                ft.Row([connect_btn, disconnect_btn, ft.Container(width=20), self._conn_status]),
                ft.Divider(),
                stop_all_btn,
            ], spacing=10),
            padding=15,
        )

    def _scan_ports(self):
        if serial:
            ports = serial.tools.list_ports.comports()
            return [ft.DropdownOption(p.device) for p in ports] or [ft.DropdownOption("COM7")]
        return [ft.DropdownOption("COM7")]

    def _refresh_ports(self):
        self._port_dropdown.options = self._scan_ports()
        self.page.update()

    def _connect_motor(self):
        port = self._port_dropdown.value or "COM7"
        try:
            baud = int(self._baud_input.value)
        except (ValueError, TypeError):
            baud = 9600
        self.motor_ctrl = MotorController(port=port, baudrate=baud)
        ok = self.motor_ctrl.connect()
        if ok:
            self._conn_status.value = f"Connected ({port})"
            self._conn_status.color = "#4CAF50"
        else:
            self._conn_status.value = f"Failed ({port})"
            self._conn_status.color = "red"
            self.motor_ctrl = None
        self.page.update()

    def _disconnect_motor(self):
        if self.motor_ctrl:
            self.motor_ctrl.stop_all(immediate=True)
            self.motor_ctrl.disconnect()
            self.motor_ctrl = None
        self._conn_status.value = "Disconnected"
        self._conn_status.color = "red"
        self.page.update()

    def _emergency_stop(self):
        if self.motor_ctrl:
            self.motor_ctrl.stop_all(immediate=True)
        self.schedule_running = False
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

    def _on_step_change(self, motor_idx, step_idx):
        self._update_distance(motor_idx, step_idx)
        self.page.update()

    def _update_distance(self, motor_idx, step_idx):
        slot = self.steps[motor_idx][step_idx]
        if not slot['speed'] or not slot['dur'] or not slot['dist_text']:
            return
        try:
            spd = int(float(slot['speed'].value))
        except (ValueError, TypeError):
            spd = 0
        dur_h = self._parse_duration_to_hours(slot['dur'].value)
        if spd > 0 and dur_h > 0:
            slot['dist_text'].value = self._calc_distance(spd, dur_h, MOTOR_TYPES[motor_idx])
        else:
            slot['dist_text'].value = "-"

    def _toggle_speed_mode(self):
        self.speed_mode = "high" if self.speed_mode == "low" else "low"
        self._render_schedule()
        self.page.update()

    def _manual_motor_start(self, motor_idx, direction):
        if self.schedule_running:
            return
        self.motor_running[motor_idx] = True
        self._highlight_motor(motor_idx, True)
        if self.motor_ctrl and self.motor_ctrl.connected:
            speed = self.high_speed if self.speed_mode == "high" else self.low_speed
            motor_id = MOTOR_IDS[motor_idx]
            mapped_dir = DIRECTION_MAP.get(direction, 'plus')
            try:
                self.motor_ctrl.start_motor(motor_id, mapped_dir, speed)
            except Exception as ex:
                print(f"Manual start error: {ex}")

    def _manual_motor_stop(self, motor_idx):
        if self.schedule_running:
            return
        self.motor_running[motor_idx] = False
        self._highlight_motor(motor_idx, False)
        if self.motor_ctrl and self.motor_ctrl.connected:
            motor_id = MOTOR_IDS[motor_idx]
            try:
                self.motor_ctrl.stop_motor(motor_id)
            except Exception as ex:
                print(f"Manual stop error: {ex}")

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
        dist_header = "Distance" if mt == 'linear' else "Rotation"

        for step in range(8):
            slot = self.steps[m][step]
            if not slot['speed']:
                slot['speed'] = self._make_table_input(
                    "1000", w=C_W, h=C_H,
                    on_change=lambda e, mi=m, si=step: self._on_step_change(mi, si))
                slot['dur'] = self._make_duration_input(
                    "00:00:30", w=C_W, h=C_H,
                    on_change=lambda e, mi=m, si=step: self._on_step_change(mi, si))
                slot['dir_dd'] = ft.Dropdown(
                    width=65, value=dir_opts[0], text_size=12, dense=True,
                    options=[ft.DropdownOption(d) for d in dir_opts],
                )
                slot['dist_text'] = ft.Text("-", size=11, text_align=ft.TextAlign.CENTER)
            self._update_distance(m, step)

        data_rows = []
        for step in range(8):
            slot = self.steps[m][step]
            toggle = self._make_step_toggle(
                f"S{step + 1}", self.step_enabled[m][step],
                lambda e, mi=m, si=step: self._toggle_step(mi, si))
            data_rows.append([
                toggle, slot['speed'], slot['dur'], slot['dir_dd'], slot['dist_text'],
            ])

        table = self._build_table(
            headers=["Step", "Speed (pps)", "Duration", "Dir", dist_header],
            data_rows=data_rows,
            col_widths=[50, C_W, C_W, 65, 90],
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

    def _render_manual_mode(self):
        spd_bg = "#ff6600" if self.speed_mode == "high" else "#2196F3"
        spd_label = f"HIGH ({self.high_speed})" if self.speed_mode == "high" else f"LOW ({self.low_speed})"
        speed_btn = ft.Container(
            content=ft.Text(spd_label, size=14, weight=ft.FontWeight.BOLD,
                            color="#ffffff", text_align=ft.TextAlign.CENTER),
            width=180, height=40, bgcolor=spd_bg, border_radius=8,
            alignment=ft.Alignment(0, 0),
            on_click=lambda e: self._toggle_speed_mode(),
        )
        self.schedule_content.controls.append(
            ft.Row([speed_btn], alignment=ft.MainAxisAlignment.CENTER))
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
        stop_btn = ft.ElevatedButton("STOP ALL", bgcolor="#ff0000", color="white",
                                     on_click=lambda e: self._emergency_stop())
        self.schedule_content.controls.append(
            ft.Row([stop_btn], alignment=ft.MainAxisAlignment.CENTER))

        self._status_text = ft.Text("", size=12)
        self.schedule_content.controls.append(self._status_text)

    # ──────────────────────────────────────────────
    # Graph
    # ──────────────────────────────────────────────

    def _render_trend(self) -> str:
        if not plt:
            return ""

        fig, ax = plt.subplots(figsize=(7, 3.5), dpi=100)

        total_h = self._get_total_duration(self.selected_motor)
        total_s = max(int(total_h * 3600), 10)

        if total_s < 60:
            t_unit, t_div = "s", 1.0
        elif total_s < 3600:
            t_unit, t_div = "min", 60.0
        else:
            t_unit, t_div = "h", 3600.0

        pts = min(total_s, 3600)
        sched_t_h = [i * total_h / pts for i in range(pts + 1)]
        sched_t_disp = [t * 3600 / t_div for t in sched_t_h]
        m = self.selected_motor
        sched_vals = [self._get_speed_at(m, t) for t in sched_t_h]

        ax.plot(sched_t_disp, sched_vals, '--', color='#2196F3', linewidth=1.5, alpha=0.7,
                label='Schedule')

        if self.history[m]:
            n = len(self.history[m])
            elapsed_s = n
            hist_t = [i / t_div for i in range(n)]
            ax.plot(hist_t, self.history[m], '-', color='#FF5722', linewidth=2, label='Actual')

        max_v = max(sched_vals) if sched_vals else 1000
        if self.history[m]:
            max_v = max(max_v, max(self.history[m]))
        ax.set_ylim(0, max_v * 1.2)
        ax.set_xlim(0, total_s / t_div)
        ax.set_xlabel(f"Time ({t_unit})", fontsize=13)
        ax.set_ylabel("Speed (pps)", fontsize=13)
        ax.set_title(f"{MOTOR_LABELS[m]}", fontsize=14, fontweight='bold')
        ax.tick_params(axis='both', labelsize=12)
        ax.legend(loc="upper right", fontsize=11)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()

        buf = io.BytesIO()
        fig.savefig(buf, format='png')
        plt.close(fig)
        buf.seek(0)
        return f"data:image/png;base64,{base64.b64encode(buf.read()).decode('ascii')}"

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

    def _get_speed_at(self, motor_idx, t_hours) -> float:
        steps_list = []
        for i, slot in enumerate(self.steps[motor_idx]):
            if not self.step_enabled[motor_idx][i]:
                continue
            sp_f = slot.get('speed')
            df = slot.get('dur')
            try:
                spd = float(sp_f.value) if sp_f and sp_f.value else 0
            except (ValueError, TypeError):
                spd = 0
            dur = self._parse_duration_to_hours(df.value) if df and df.value else 0.0
            steps_list.append((spd, dur))

        if not steps_list:
            return 0

        cum = 0.0
        prev = steps_list[0][0]
        for spd, dur in steps_list:
            if dur <= 0:
                prev = spd
                continue
            if t_hours < cum + dur:
                frac = (t_hours - cum) / dur
                return prev + (spd - prev) * max(0, min(1, frac))
            cum += dur
            prev = spd
        return steps_list[-1][0]

    def _get_direction_at(self, motor_idx, t_hours) -> str:
        steps_list = []
        for i, slot in enumerate(self.steps[motor_idx]):
            if not self.step_enabled[motor_idx][i]:
                continue
            df = slot.get('dur')
            dd = slot.get('dir_dd')
            dur = self._parse_duration_to_hours(df.value) if df and df.value else 0.0
            d = dd.value if dd else '+'
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

    def _apply_schedule(self):
        self.history = [[] for _ in range(4)]
        self.schedule_time = 0.0
        self.schedule_running = False
        self.trend_image.src = self._render_trend()
        self.trend_image.update()
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
        self.schedule_running = True
        self.start_btn.text = "Stop"
        self._status_text.value = "Running..."
        self._status_text.color = "#4CAF50"
        self.page.update()
        self.page.run_task(self._schedule_loop)

    async def _schedule_loop(self):
        while self.schedule_running:
            try:
                self.schedule_time += 1.0 / 3600.0

                total_dur = self._get_total_duration(self.selected_motor)
                if total_dur > 0 and self.schedule_time >= total_dur:
                    self.schedule_time = total_dur
                    if self.motor_ctrl:
                        self.motor_ctrl.stop_all()
                    self.schedule_running = False
                    self.start_btn.text = "Start"
                    self._status_text.value = "Schedule complete"
                    self._status_text.color = "#2196F3"
                    self.trend_image.src = self._render_trend()
                    self.trend_image.update()
                    self.page.update()
                    break

                for mi in range(4):
                    spd = int(self._get_speed_at(mi, self.schedule_time))
                    self.history[mi].append(spd)

                    if self.motor_ctrl and self.motor_ctrl.connected:
                        motor_id = MOTOR_IDS[mi]
                        direction = self._get_direction_at(mi, self.schedule_time)
                        mapped_dir = DIRECTION_MAP.get(direction, 'plus')
                        try:
                            if spd > 0:
                                self.motor_ctrl.start_motor(motor_id, mapped_dir, spd)
                            else:
                                self.motor_ctrl.stop_motor(motor_id)
                        except Exception as ex:
                            print(f"Motor {motor_id} error: {ex}")

                self.trend_image.src = self._render_trend()
                self.trend_image.update()

            except Exception as ex:
                print(f"Schedule loop error: {ex}")

            await asyncio.sleep(1.0)

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

        self._build_numpad()
        self._build_duration_pad()

        placeholder_b64 = (
            "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAAC0lE"
            "QVQIHWNgAAIABQABNjN9GQAAAAlwSFlzAAAWJQAAFiUBSVIk8AAAAAtJREFUCB1jYAACAAAFAAGbfEHV"
            "AAAAAElFTkSuQmCC"
        )
        self.trend_image = ft.Image(src=placeholder_b64, width=900, height=500)

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
            content=self.trend_image,
            expand=3, bgcolor="#ffffff", border_radius=8, padding=5,
        )

        layout = ft.Row([left_panel, right_panel], expand=True, spacing=4)

        self._switch_view("schedule")

        page.add(ft.Stack([layout, self._numpad_overlay, self._durpad_overlay], expand=True))

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
