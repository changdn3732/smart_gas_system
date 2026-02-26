import flet as ft
import asyncio
import random
import io
import base64
from collections import deque
import matplotlib
# use non-interactive backend
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import math
try:
    import serial
except Exception:
    serial = None

# ensure parent workspace folder is on sys.path so local packages (simulators/) can be imported
import os
import sys
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from devices.device_service import DeviceService
from data.data_service import DataService
from devices.gas_controller import ALICAT_GAS_LIST, GAS_TABLE

class SchedulerApp:
    class _SquareButton:
        def __init__(self, label, on_click=None, width=None, bgcolor=None, color=None):
            # default to navy background and white text unless specified
            if bgcolor is None:
                bgcolor = "#003366"
            if color is None:
                color = "#ffffff"
            self._text = ft.Text(label, color=color)
            style_kwargs = {}
            if width:
                style_kwargs['width'] = width
            self.control = ft.Container(content=ft.Row([self._text], alignment=ft.MainAxisAlignment.CENTER), padding=8, bgcolor=bgcolor, border=ft.Border.all(1, "#003366"), **style_kwargs)
            if on_click:
                self.control.on_click = lambda e: on_click(e)

        @property
        def text(self):
            return getattr(self._text, 'value', None) or ''

        @text.setter
        def text(self, v):
            try:
                self._text.value = v
            except Exception:
                pass
            try:
                self.control.update()
            except Exception:
                pass
    
    def _emergency_stop(self):
        try:
            for i in range(4):
                self.device_service.set_gas(i, 1, 0.0)

            self.conn_status.value = "Emergency Stop (All Gas = 0)"
            self.conn_status.color = "red"

            self.page.update()

        except Exception as ex:
            print("emergency error:", ex)

    def _on_gas_type_change(self, ch: int, gas: str):
        self.channel_gases[ch] = gas
        print(f"CH{ch+1} Gas changed to {gas}")

    def _apply_gas_types(self):
        if not self.device_service:
            self.device_service = DeviceService()
        for ch in range(4):
            gas_name = self.channel_gases[ch]
            gas_index = self.gas_name_to_index.get(gas_name)
            if gas_index is None:
                print(f"CH{ch+1}: unknown gas '{gas_name}'")
                continue
            try:
                ok = self.device_service.write_gas_type(ch, gas_index)
                print(f"CH{ch+1} gas type → {gas_name} (index {gas_index}): {'OK' if ok else 'FAIL'}")
            except Exception as ex:
                print(f"CH{ch+1} gas type 전송 오류: {ex}")

    def _toggle_mixing_channel(self, ch: int):
        # 선택 상태 토글
        self.mixing_selected[ch] = not self.mixing_selected[ch]

        # UI 다시 그리기
        self._render_schedule_content()
        self.page.update()

    def _set_control_mode(self, mode: str):
        self.control_mode = mode
        self._render_schedule_content()
        self.page.update()


    def _select_gas_schedule_channel(self, ch: int):
        self.selected_gas_channel = ch
        self._render_schedule_content()
        self.page.update()    


    def _on_gas_step_change(self, ch_idx: int, step_idx: int):
        """
        Gas step 값이 변경되었을 때 호출.
        현재는 그래프 즉시 반영용으로 트렌드만 다시 그림.
        """

        try:
            # 스케줄 시간 초기화해서 즉시 반영
            self.schedule_time = 0.0

            # 현재 타겟으로 히스토리 초기화
            targets = self._get_gas_targets(0.0)

            for i in range(4):
                init_val = targets[i] if targets and i < len(targets) else 0.0
                self.history[i].clear()
                self.history[i].extend([init_val] * 600)

            # 트렌드 다시 렌더
            self.trend_image.src = self._render_trend()
            self.trend_image.update()
            self.page.update()

        except Exception as e:
            print("gas step change error:", e)

    def _make_sq_button(self, label, on_click=None, width=None, bgcolor=None, color=None):
        return SchedulerApp._SquareButton(label, on_click=on_click, width=width, bgcolor=bgcolor, color=color)

    def __init__(self, page: ft.Page):
        self.page = page
        page.title = "Temperature & Gas Scheduler"
        # default device/connection state (ensure available before settings opened)
        self.conn_status = ft.Text("Not connected", color="red")
        self.device_config = {"ut32a_id": 1, "alicat_start_id": 2}
        self.selected_gas_channel = 0   # 기본 CH1 선택
        # state
        self.schedule_mode = "temp"  # 'temp' or 'gas'
        # initialize 8 temp steps + enable state
        self.temp_steps = [
            {"temp_field": None, "dur_field": None, "rate_text": None} for _ in range(8)
        ]
        self.temp_step_enabled = [True, True, True, True, False, False, False, False]
        # gas control state
        self.control_mode = "Mixing Mode"
        self.selected_gas_channel = 0
        self.active_gas_channel = None
        # build UI
        self.schedule_panel = self._build_schedule_panel()
        self.available_gases = [g[1] for g in ALICAT_GAS_LIST]
        self.gas_name_to_index = {g[1]: g[0] for g in ALICAT_GAS_LIST}
        self.device_service = None
        self.channel_gases = ["N2", "N2", "N2", "N2"]
        self.gas_type_dropdowns = [None, None, None, None]
        # Prepare in-memory history for 4 channels (keep up to 600 samples)
        self.history = {i: deque([0.0] * 600, maxlen=600) for i in range(4)}
        # single measured series that will approximate the schedule (keep up to 600 samples)
        self.measured = deque([25.0] * 600, maxlen=600)
        # gas steps storage: for 4 channels, each has up to 8 steps
        self.gas_steps = [
            [{"setpoint": None, "dur_field": None, "rate_text": None} for _ in range(8)] for _ in range(4)
        ]
        self.gas_step_enabled = [
            [True, True, True, True, False, False, False, False] for _ in range(4)
        ]
        # Mixing mode용 공통 스케줄
        self.mixing_steps = [
            {"setpoint": None, "dur_field": None} for _ in range(8)
        ]
        self.mixing_step_enabled = [True, True, True, True, False, False, False, False]

        # Mixing mode 채널 선택 상태
        self.mixing_selected = [False, False, False, False]  # 기본 선택 안함
        # per-channel simple controls for the left control panel (gas selector + setpoint)
        self.channel_controls = [
            {"gas_dd": None, "sp_field": None} for _ in range(4)
        ]
        # active gas channel to run individually (None = use averaged behavior)
        self.active_gas_channel = None
        # schedule time in hours (advances with each sample)
        self.schedule_time = 0.0

        # build touch numpad
        self._build_numpad()

        # initial 1x1 PNG placeholder
        placeholder_b64 = (
            "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVQI12NgYAAAAAMAAWgmWQ0AAAAASUVORK5CYII="
        )
        self.trend_image = ft.Image(src=f"data:image/png;base64,{placeholder_b64}", width=900, height=500)
        self.trend_placeholder = ft.Container(
            content=self.trend_image,
            padding=12,
            border=ft.Border.all(1, "#e0e0e0"),
        )

        # runtime control for trend updates
        self.trend_running = False
        self.trend_run_button = self._make_sq_button("Start", on_click=self._toggle_trend)

        # status table placeholder
        self.status_table = None

        # zoom/window controls (seconds == samples)
        self.window_samples = 60
        self.zoom_label = ft.Text(f"Window: {self.window_samples}s")
        self.zoom_out_btn = self._make_sq_button("-", on_click=self._zoom_out, width=40)
        self.zoom_in_btn = self._make_sq_button("+", on_click=self._zoom_in, width=40)

        # start async update loop to inject test data and update image (idle until started)
        try:
            page.run_task(self._trend_loop_async)
        except Exception:
            # fallback: schedule via asyncio
            asyncio.get_event_loop().create_task(self._trend_loop_async())

        # layout: left 50% (expand=5), right 50% (expand=5)
        layout = ft.Row([
            ft.Container(content=self.schedule_panel, expand=5, padding=12),
            ft.Container(width=12),
            ft.Container(content=ft.Column([
                ft.Text("Real-time Trends", size=16, weight=ft.FontWeight.BOLD),
                self.trend_placeholder,
                ft.Container(height=8),
                ft.Row([
                    self.zoom_out_btn.control,
                    ft.Container(width=8),
                    self.zoom_label,
                    ft.Container(width=8),
                    self.zoom_in_btn.control,
                    ft.Container(width=24),
                    self.trend_run_button.control,
                ], alignment=ft.MainAxisAlignment.CENTER),
            ]), expand=5, padding=12),
        ], expand=True)

        page.add(ft.Stack([layout, self._numpad_overlay], expand=True))

    def _build_schedule_panel(self):
        temp_btn = self._make_sq_button("Temperature", on_click=lambda e: self._switch_schedule_mode("temp"))
        gas_btn = self._make_sq_button("Gas", on_click=lambda e: self._switch_schedule_mode("gas"))
        settings_btn = self._make_sq_button("Settings", on_click=self._open_settings)

        # content column that will be re-rendered
        self.schedule_content = ft.Column([], spacing=8)
        self._render_schedule_content()

        return ft.Container(
            content=ft.Column([
                ft.Row([temp_btn.control, ft.Container(width=8), gas_btn.control, ft.Container(width=8), settings_btn.control]),
                ft.Divider(height=8),
                self.schedule_content,
            ], spacing=8),
            bgcolor="#ffffff",
            border_radius=8,
            padding=12,
        )

    def _switch_schedule_mode(self, mode: str):
        self.schedule_mode = mode
        self._render_schedule_content()
        # refresh trend image to reflect selected data channels
        try:
            self.trend_image.src = self._render_trend()
            self.trend_image.update()
            self.page.update()
        except Exception:
            try:
                self.page.update()
            except Exception:
                pass
        # reset schedule time so the displayed data follows the schedule from start
        self.schedule_time = 0.0
        # reset measured/history depending on mode
        if self.schedule_mode == 'temp':
            try:
                init_val = self._get_schedule_target(0.0)
            except Exception:
                init_val = 25.0
            self.measured = deque([init_val] * self.measured.maxlen, maxlen=self.measured.maxlen)
        else:
            # initialize per-channel history from gas targets
            try:
                targets = self._get_gas_targets(0.0)
            except Exception:
                targets = [0.0] * 4
            for i in range(4):
                v = targets[i] if targets and i < len(targets) and targets[i] is not None else 0.0
                self.history[i] = deque([v] * self.history[i].maxlen, maxlen=self.history[i].maxlen)

    def _make_table_cell(self, content, w=80, h=34, bg="#ffffff", header=False,
                         border_top=True, border_bottom=True, border_left=True, border_right=True):
        sides = ft.BorderSide(1, "#b0b0b0")
        none_side = ft.BorderSide(0, "transparent")
        border = ft.Border(
            top=sides if border_top else none_side,
            bottom=sides if border_bottom else none_side,
            left=sides if border_left else none_side,
            right=sides if border_right else none_side,
        )
        if isinstance(content, str):
            content = ft.Text(content, size=11,
                              weight=ft.FontWeight.BOLD if header else None,
                              text_align=ft.TextAlign.CENTER)
        return ft.Container(content=content, width=w, height=h,
                            bgcolor=bg, border=border,
                            alignment=ft.Alignment(0, 0), padding=0)

    def _make_table_input(self, value="0.0", w=80, h=34, on_change=None):
        display = ft.Text(value, size=12, text_align=ft.TextAlign.CENTER)
        cell = ft.Container(
            content=display,
            width=w, height=h,
            alignment=ft.Alignment(0, 0),
            bgcolor="#ffffff",
            border_radius=2,
            on_click=lambda e: self._show_numpad(cell),
        )
        cell.value = value
        cell.on_change = on_change
        cell._display = display
        return cell

    # ─── Numeric Keypad (touch panel) ───

    def _build_numpad(self):
        self._numpad_target = None
        self._numpad_value = ""
        self._numpad_open = False
        self._numpad_display = ft.Text(
            "0", size=32, weight=ft.FontWeight.BOLD,
            text_align=ft.TextAlign.RIGHT)

        def _kb(label, w=70, bg="#f5f5f5", fg="#000000"):
            return ft.Container(
                content=ft.Text(label, size=22, weight=ft.FontWeight.BOLD,
                                color=fg, text_align=ft.TextAlign.CENTER),
                width=w, height=60, bgcolor=bg, border_radius=8,
                alignment=ft.Alignment(0, 0),
                border=ft.Border.all(1, "#cccccc"),
                on_click=lambda e, k=label: self._numpad_key(k),
            )

        ok_btn = ft.Container(
            content=ft.Text("OK", size=22, weight=ft.FontWeight.BOLD,
                            color="#ffffff", text_align=ft.TextAlign.CENTER),
            width=70, height=60, bgcolor="#003366", border_radius=8,
            alignment=ft.Alignment(0, 0),
            on_click=lambda e: self._numpad_confirm(),
        )

        cancel_btn = ft.Container(
            content=ft.Text("Cancel", size=16, color="#666666",
                            text_align=ft.TextAlign.CENTER),
            width=310, height=40, bgcolor="#eeeeee", border_radius=8,
            alignment=ft.Alignment(0, 0),
            on_click=lambda e: self._numpad_cancel(),
        )

        pad_card = ft.Container(
            content=ft.Column([
                ft.Text("\uc22b\uc790 \uc785\ub825", size=18, weight=ft.FontWeight.BOLD),
                ft.Container(height=4),
                ft.Container(
                    content=self._numpad_display,
                    bgcolor="#f9f9f9", border=ft.Border.all(2, "#003366"),
                    border_radius=8, padding=12,
                    width=310, alignment=ft.Alignment(1, 0),
                ),
                ft.Container(height=8),
                ft.Row([_kb("7"), _kb("8"), _kb("9"), _kb("\u232b", bg="#ffe0e0")],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Row([_kb("4"), _kb("5"), _kb("6"), _kb("C", bg="#ffe0e0")],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Row([_kb("1"), _kb("2"), _kb("3"), _kb("\u00b1", bg="#e0e0ff")],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Row([_kb("0", w=148), _kb("."), ok_btn],
                       spacing=8, alignment=ft.MainAxisAlignment.CENTER),
                ft.Container(height=4),
                cancel_btn,
            ], spacing=8, horizontal_alignment=ft.CrossAxisAlignment.CENTER),
            width=360, bgcolor="#ffffff", border_radius=12,
            padding=20, border=ft.Border.all(2, "#003366"),
        )

        self._numpad_overlay = ft.Container(
            content=pad_card,
            bgcolor="rgba(0,0,0,0.4)",
            alignment=ft.Alignment(0, 0),
            expand=True,
            visible=False,
            on_click=lambda e: self._numpad_cancel(),
        )

    def _show_numpad(self, target_field):
        if self._numpad_open:
            return
        self._numpad_open = True
        self._numpad_target = target_field
        self._numpad_value = target_field.value or ""
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

    # ─── Table helpers ───

    def _build_table(self, headers, data_rows, col_widths, row_height=34):
        """Build an Excel-style table with proper borders and center alignment."""
        H_BG = "#e8eef4"
        L_BG = "#f0f4f8"
        n_cols = len(headers)
        n_rows = len(data_rows)

        def cell_borders(r, c, total_r):
            return dict(
                border_top=(r == 0),
                border_bottom=True,
                border_left=(c == 0),
                border_right=True,
            )

        header_cells = []
        for c, hdr in enumerate(headers):
            header_cells.append(self._make_table_cell(
                hdr, w=col_widths[c], h=row_height, bg=H_BG, header=True,
                **cell_borders(0, c, n_rows + 1)))
        rows = [ft.Row(header_cells, spacing=0, alignment=ft.MainAxisAlignment.CENTER)]

        for r, row_data in enumerate(data_rows):
            row_cells = []
            for c, cell_content in enumerate(row_data):
                is_label = (c == 0)
                bg = L_BG if is_label else "#ffffff"
                row_cells.append(self._make_table_cell(
                    cell_content, w=col_widths[c], h=row_height,
                    bg=bg, header=is_label,
                    **cell_borders(r + 1, c, n_rows + 1)))
            rows.append(ft.Row(row_cells, spacing=0, alignment=ft.MainAxisAlignment.CENTER))

        return ft.Container(
            content=ft.Column(rows, spacing=0, horizontal_alignment=ft.CrossAxisAlignment.CENTER),
            border=ft.Border.all(2, "#666666"), border_radius=4,
        )

    def _toggle_temp_step(self, idx):
        self.temp_step_enabled[idx] = not self.temp_step_enabled[idx]
        self._render_schedule_content()
        self.page.update()

    def _toggle_mixing_step(self, idx):
        self.mixing_step_enabled[idx] = not self.mixing_step_enabled[idx]
        self._render_schedule_content()
        self.page.update()

    def _toggle_gas_step(self, ch, idx):
        self.gas_step_enabled[ch][idx] = not self.gas_step_enabled[ch][idx]
        self._render_schedule_content()
        self.page.update()

    def _make_step_toggle(self, label, enabled, on_click):
        bg = "#003366" if enabled else "#dddddd"
        fg = "#ffffff" if enabled else "#888888"
        return ft.Container(
            content=ft.Text(label, size=10, weight=ft.FontWeight.BOLD,
                            color=fg, text_align=ft.TextAlign.CENTER),
            width=46, height=28, bgcolor=bg, border_radius=4,
            alignment=ft.Alignment(0, 0),
            on_click=on_click,
        )

    def _render_schedule_content(self):
        self.schedule_content.controls.clear()
        C_W = 90
        C_H = 34

        if self.schedule_mode == "temp":
            self.schedule_content.controls.append(
                ft.Text("Temperature Scheduling", size=14, weight=ft.FontWeight.BOLD))

            for i in range(8):
                if not self.temp_steps[i]["temp_field"]:
                    self.temp_steps[i]["temp_field"] = self._make_table_input(
                        "25.0", w=C_W, h=C_H,
                        on_change=lambda e, idx=i: self._on_temp_or_dur_change(idx))
                    self.temp_steps[i]["dur_field"] = self._make_table_input(
                        "0.5", w=C_W, h=C_H,
                        on_change=lambda e, idx=i: self._on_temp_or_dur_change(idx))
                    self.temp_steps[i]["rate_text"] = ft.Text(
                        "-", size=10, text_align=ft.TextAlign.CENTER)

            data_rows = []
            for i in range(8):
                toggle = self._make_step_toggle(
                    f"S{i+1}", self.temp_step_enabled[i],
                    lambda e, idx=i: self._toggle_temp_step(idx))
                data_rows.append([
                    toggle,
                    self.temp_steps[i]["temp_field"],
                    self.temp_steps[i]["dur_field"],
                    self.temp_steps[i]["rate_text"],
                ])

            table = self._build_table(
                headers=["Step", "Temp (°C)", "Duration (h)", "Rate (°/h)"],
                data_rows=data_rows,
                col_widths=[50, C_W, C_W, C_W],
                row_height=C_H,
            )
            self.schedule_content.controls.append(
                ft.Row([table], alignment=ft.MainAxisAlignment.CENTER))
            self._recompute_temp_rates()

            btn_apply = self._make_sq_button("Apply Temp Schedule", on_click=lambda e: print("Apply temp schedule"))
            btn_start = self._make_sq_button("Start Schedule", on_click=lambda e: print("Start temp schedule"))
            self.schedule_content.controls.append(ft.Row([
                btn_apply.control, ft.Container(width=12), btn_start.control,
            ], alignment=ft.MainAxisAlignment.CENTER))

        else:
            self.schedule_content.controls.append(
                ft.Text("Gas Control Mode", size=14, weight=ft.FontWeight.BOLD))

            mode_row = ft.Row([
                self._make_sq_button("Mixing Mode",
                    on_click=lambda e: self._set_control_mode("Mixing Mode"),
                    bgcolor="#003366" if self.control_mode == "Mixing Mode" else "#cccccc").control,
                ft.Container(width=10),
                self._make_sq_button("Manual Mode",
                    on_click=lambda e: self._set_control_mode("Manual Mode"),
                    bgcolor="#003366" if self.control_mode == "Manual Mode" else "#cccccc").control,
            ], alignment=ft.MainAxisAlignment.CENTER)
            self.schedule_content.controls.append(mode_row)
            self.schedule_content.controls.append(ft.Divider())

            gas_config_row = ft.Row(alignment=ft.MainAxisAlignment.CENTER, spacing=6)
            for ch in range(4):
                if self.gas_type_dropdowns[ch] is None:
                    self.gas_type_dropdowns[ch] = ft.Dropdown(
                        width=110,
                        value=self.channel_gases[ch],
                        options=[ft.DropdownOption(g) for g in self.available_gases],
                        on_select=lambda e, c=ch: self._on_gas_type_change(c, e.control.value),
                        text_size=12,
                        dense=True,
                    )
                else:
                    self.gas_type_dropdowns[ch].value = self.channel_gases[ch]
                gas_config_row.controls.append(
                    ft.Column([
                        ft.Text(f"CH{ch+1}", size=11, weight=ft.FontWeight.BOLD,
                                text_align=ft.TextAlign.CENTER),
                        self.gas_type_dropdowns[ch],
                    ], horizontal_alignment=ft.CrossAxisAlignment.CENTER, spacing=2)
                )
            apply_gas_btn = self._make_sq_button(
                "Apply", on_click=lambda e: self._apply_gas_types(), width=70)
            gas_config_row.controls.append(apply_gas_btn.control)
            self.schedule_content.controls.append(gas_config_row)
            self.schedule_content.controls.append(ft.Divider())

            if self.control_mode == "Mixing Mode":
                ch_row = ft.Row(alignment=ft.MainAxisAlignment.CENTER)
                for ch in range(4):
                    btn = self._make_sq_button(f"CH{ch+1}",
                        on_click=lambda e, c=ch: self._toggle_mixing_channel(c),
                        bgcolor="#003366" if self.mixing_selected[ch] else "#cccccc")
                    ch_row.controls.append(btn.control)
                    ch_row.controls.append(ft.Container(width=10))
                self.schedule_content.controls.append(ch_row)
                self.schedule_content.controls.append(ft.Divider())

                for step in range(8):
                    slot = self.mixing_steps[step]
                    if not slot["setpoint"]:
                        slot["setpoint"] = self._make_table_input("0.0", w=C_W, h=C_H)
                        slot["dur_field"] = self._make_table_input("0.5", w=C_W, h=C_H)

                data_rows = []
                for step in range(8):
                    slot = self.mixing_steps[step]
                    toggle = self._make_step_toggle(
                        f"S{step+1}", self.mixing_step_enabled[step],
                        lambda e, idx=step: self._toggle_mixing_step(idx))
                    data_rows.append([toggle, slot["setpoint"], slot["dur_field"]])

                table = self._build_table(
                    headers=["Step", "SP (sccm)", "Duration (h)"],
                    data_rows=data_rows,
                    col_widths=[50, C_W, C_W],
                    row_height=C_H,
                )
                self.schedule_content.controls.append(
                    ft.Row([table], alignment=ft.MainAxisAlignment.CENTER))

            else:
                ch_row = ft.Row(alignment=ft.MainAxisAlignment.CENTER)
                for ch in range(4):
                    btn = self._make_sq_button(f"CH{ch+1}",
                        on_click=lambda e, c=ch: self._select_gas_schedule_channel(c),
                        bgcolor="#003366" if self.selected_gas_channel == ch else "#cccccc")
                    ch_row.controls.append(btn.control)
                    ch_row.controls.append(ft.Container(width=10))
                self.schedule_content.controls.append(ch_row)
                self.schedule_content.controls.append(ft.Divider())

                ch = self.selected_gas_channel
                for step in range(8):
                    slot = self.gas_steps[ch][step]
                    if not slot["setpoint"]:
                        slot["setpoint"] = self._make_table_input("0.0", w=C_W, h=C_H)
                        slot["dur_field"] = self._make_table_input("0.5", w=C_W, h=C_H)

                data_rows = []
                for step in range(8):
                    slot = self.gas_steps[ch][step]
                    toggle = self._make_step_toggle(
                        f"S{step+1}", self.gas_step_enabled[ch][step],
                        lambda e, c=ch, idx=step: self._toggle_gas_step(c, idx))
                    data_rows.append([toggle, slot["setpoint"], slot["dur_field"]])

                table = self._build_table(
                    headers=["Step", "SP (sccm)", "Duration (h)"],
                    data_rows=data_rows,
                    col_widths=[50, C_W, C_W],
                    row_height=C_H,
                )
                self.schedule_content.controls.append(
                    ft.Row([table], alignment=ft.MainAxisAlignment.CENTER))               
    def _on_temp_or_dur_change(self, idx: int):
                    try:
                        self._recompute_temp_rates()
                        self.page.update()
                    except Exception:
                        pass




    def _scan_ports(self):
        """Return list of (port_name, description) for available serial ports."""
        try:
            from serial.tools import list_ports
            ports = sorted(list_ports.comports(), key=lambda p: p.device)
            return [(p.device, p.description) for p in ports]
        except Exception:
            return []

    def _refresh_ports(self):
        """Rescan serial ports and update the dropdown options."""
        port_options = self._scan_ports()
        opts = [ft.DropdownOption(key=p, text=f"{p}  {desc}") for p, desc in port_options]
        self.temp_port_field.options = opts
        self.gas_port_field.options = opts
        self.page.update()

    def _build_device_indicator(self, label):
        icon = ft.Icon(ft.Icons.CIRCLE, size=14, color="red")
        text = ft.Text(label, size=13, width=160)
        status_text = ft.Text("Not checked", size=12, color="#999999")
        return icon, text, status_text

    def _open_settings(self, e):
        try:
            if not hasattr(self, "settings_view"):
                self.conn_status = ft.Text("Not connected", color="red")
                self.device_type = ft.Dropdown(options=[ft.DropdownOption("Simulator"), ft.DropdownOption("Serial")], value="Simulator", width=180)

                port_options = self._scan_ports()

                import platform
                _is_win = platform.system() == "Windows"
                _def_temp = "COM7" if _is_win else "/dev/ttyUSB0"
                _def_gas = "COM5" if _is_win else "/dev/ttyUSB1"

                _temp_val = _def_temp if _def_temp in [p for p, _ in port_options] else (port_options[0][0] if port_options else "")
                _gas_val = _def_gas if _def_gas in [p for p, _ in port_options] else (port_options[1][0] if len(port_options) > 1 else (port_options[0][0] if port_options else ""))

                self.temp_port_field = ft.Dropdown(
                    label="Temp Port (RS-485)", width=200,
                    value=_temp_val,
                    options=[ft.DropdownOption(key=p, text=f"{p}  {desc}") for p, desc in port_options],
                )
                self._temp_baud_display = ft.Text("19200", size=14, text_align=ft.TextAlign.CENTER)
                self.temp_baud_field = ft.Container(
                    content=ft.Column([
                        ft.Text("Baud", size=10, color="#666666"),
                        self._temp_baud_display,
                    ], spacing=0, horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                    width=100, height=48, alignment=ft.Alignment(0, 0),
                    border=ft.Border.all(1, "#cccccc"), border_radius=4, bgcolor="#ffffff",
                    on_click=lambda e: self._show_numpad(self.temp_baud_field),
                )
                self.temp_baud_field.value = "19200"
                self.temp_baud_field._display = self._temp_baud_display
                self.temp_baud_click = self.temp_baud_field

                self.gas_port_field = ft.Dropdown(
                    label="Gas Port (RS-232)", width=200,
                    value=_gas_val,
                    options=[ft.DropdownOption(key=p, text=f"{p}  {desc}") for p, desc in port_options],
                )
                self._gas_baud_display = ft.Text("19200", size=14, text_align=ft.TextAlign.CENTER)
                self.gas_baud_field = ft.Container(
                    content=ft.Column([
                        ft.Text("Baud", size=10, color="#666666"),
                        self._gas_baud_display,
                    ], spacing=0, horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                    width=100, height=48, alignment=ft.Alignment(0, 0),
                    border=ft.Border.all(1, "#cccccc"), border_radius=4, bgcolor="#ffffff",
                    on_click=lambda e: self._show_numpad(self.gas_baud_field),
                )
                self.gas_baud_field.value = "19200"
                self.gas_baud_field._display = self._gas_baud_display
                self.gas_baud_click = self.gas_baud_field

                self._port_refresh_btn = self._make_sq_button(
                    "Refresh Ports", on_click=lambda ev: self._refresh_ports(), width=120)

                connect_btn = self._make_sq_button("Connect Device", on_click=lambda ev: self._toggle_connection())

                # 개별 장치 상태 인디케이터 (UT32A x1 + Gas x4)
                device_labels = [
                    "UT32A  (ID: 1, Temp)",
                    "Gas CH1 (ID: 2, MFC)",
                    "Gas CH2 (ID: 3, MFC)",
                    "Gas CH3 (ID: 4, MFC)",
                    "Gas CH4 (ID: 5, MFC)",
                ]
                self.dev_icons = []
                self.dev_status_texts = []
                status_rows = []

                for label in device_labels:
                    icon, name_text, status_text = self._build_device_indicator(label)
                    self.dev_icons.append(icon)
                    self.dev_status_texts.append(status_text)
                    status_rows.append(
                        ft.Row([icon, name_text, status_text], spacing=8)
                    )

                status_section = ft.Container(
                    content=ft.Column(
                        [ft.Text("Device Status", size=14, weight=ft.FontWeight.BOLD)]
                        + status_rows,
                        spacing=6,
                    ),
                    padding=ft.padding.all(12),
                    bgcolor="#f9f9f9",
                    border_radius=8,
                    border=ft.Border.all(1, "#e0e0e0"),
                )

                # Setpoint 직접 전송
                self.sp_input_fields = []
                self.sp_result_texts = []
                sp_rows = []
                for ch in range(4):
                    sp_display = ft.Text("0.0", size=14, text_align=ft.TextAlign.CENTER)
                    sp_inp = ft.Container(
                        content=ft.Column([
                            ft.Text(f"CH{ch+1} Setpoint", size=10, color="#666666"),
                            sp_display,
                        ], spacing=0, horizontal_alignment=ft.CrossAxisAlignment.CENTER),
                        width=140, height=48, alignment=ft.Alignment(0, 0),
                        border=ft.Border.all(1, "#cccccc"), border_radius=4, bgcolor="#ffffff",
                        on_click=lambda e, f=None: self._show_numpad(f),
                    )
                    sp_inp.value = "0.0"
                    sp_inp._display = sp_display
                    sp_inp.on_change = None
                    sp_inp.on_click = lambda e, f=sp_inp: self._show_numpad(f)
                    sp_result = ft.Text("", size=12, color="#999999")
                    send_btn = ft.ElevatedButton(
                        f"Send",
                        on_click=lambda _, c=ch: self._send_setpoint_from_settings(c),
                        height=36,
                    )
                    self.sp_input_fields.append(sp_inp)
                    self.sp_result_texts.append(sp_result)
                    sp_rows.append(ft.Row([
                        ft.Text(f"Gas CH{ch+1}", width=70, weight=ft.FontWeight.W_500),
                        sp_inp, send_btn, sp_result,
                    ], spacing=8))

                setpoint_section = ft.Container(
                    content=ft.Column(
                        [ft.Text("Setpoint Control", size=14, weight=ft.FontWeight.BOLD)]
                        + sp_rows,
                        spacing=8,
                    ),
                    padding=ft.padding.all(12),
                    bgcolor="#f9f9f9",
                    border_radius=8,
                    border=ft.Border.all(1, "#e0e0e0"),
                )

                # Gas Configuration
                gas_config_section = ft.Column(
                    [ft.Text("Gas Configuration", size=14, weight=ft.FontWeight.BOLD), ft.Divider()],
                    spacing=8,
                )

                for ch in range(4):
                    dropdown = ft.Dropdown(
                        width=200,
                        value=self.channel_gases[ch],
                        options=[ft.DropdownOption(g) for g in self.available_gases],
                        on_select=lambda e, c=ch: self._on_gas_type_change(c, e.control.value),
                    )
                    gas_config_section.controls.append(ft.Row([ft.Text(f"CH{ch+1}", width=60), dropdown]))

                port_section = ft.Column([
                    ft.Text("Port Configuration", size=14, weight=ft.FontWeight.BOLD),
                    ft.Row([
                        ft.Text("Device Type:"), self.device_type,
                        ft.Container(width=16),
                        self._port_refresh_btn.control,
                    ]),
                    ft.Row([
                        self.temp_port_field, ft.Container(width=8), self.temp_baud_click,
                        ft.Container(width=20),
                        self.gas_port_field, ft.Container(width=8), self.gas_baud_click,
                    ]),
                ], spacing=8)

                self.settings_view = ft.View(route="/settings", controls=[
                    ft.AppBar(title=ft.Text("Settings"), bgcolor="#ffffff", leading=ft.TextButton("Back", on_click=self._close_settings)),
                    ft.Container(content=ft.Column([
                        port_section,
                        self.conn_status,
                        connect_btn.control,
                        ft.Divider(height=12),
                        status_section,
                        ft.Divider(height=12),
                        setpoint_section,
                        ft.Divider(height=12),
                        gas_config_section,
                    ], spacing=12, scroll=ft.ScrollMode.AUTO), padding=12)
                ])

            # push and navigate
            if self.settings_view not in self.page.views:
                self.page.views.append(self.settings_view)
            self.page.go(self.settings_view.route)
        except Exception as ex:
            print("open settings error:", ex)

    def _close_settings(self, e):
        try:
            # go back to main view
            if len(self.page.views) > 0:
                # remove settings view if present
                for v in list(self.page.views):
                    if v.route == "/settings":
                        self.page.views.remove(v)
            # navigate to default (root)
            if len(self.page.views) > 0:
                self.page.go(self.page.views[-1].route)
            else:
                self.page.go("/")
        except Exception:
            pass

    def _update_device_indicator(self, index, connected, write_only=False):
        if not hasattr(self, "dev_icons"):
            return
        if connected and write_only:
            self.dev_icons[index].color = "#FF9800"
            self.dev_status_texts[index].value = "Write-only (RX?)"
            self.dev_status_texts[index].color = "#FF9800"
        elif connected:
            self.dev_icons[index].color = "#4CAF50"
            self.dev_status_texts[index].value = "Connected"
            self.dev_status_texts[index].color = "#4CAF50"
        else:
            self.dev_icons[index].color = "#F44336"
            self.dev_status_texts[index].value = "No response"
            self.dev_status_texts[index].color = "#F44336"

    def _toggle_connection(self, e=None):
        try:
            dtype = self.device_type.value

            self.device_service = DeviceService()

            if dtype == "Simulator":
                self.device_service.connect_simulator()
                self.conn_status.value = "Connected (Simulator)"
                self.conn_status.color = "green"
                for i in range(5):
                    self._update_device_indicator(i, True)

            else:
                temp_port = (self.temp_port_field.value or "").strip()
                temp_baud = int(self.temp_baud_field.value)
                gas_port = (self.gas_port_field.value or "").strip()
                gas_baud = int(self.gas_baud_field.value)

                # 인디케이터를 "Checking..." 상태로
                ports_str = []
                if temp_port:
                    ports_str.append(f"Temp:{temp_port}")
                if gas_port:
                    ports_str.append(f"Gas:{gas_port}")
                self.conn_status.value = f"Connecting {', '.join(ports_str)}..."
                self.conn_status.color = "#666666"
                for i in range(5):
                    self.dev_icons[i].color = "#999999"
                    self.dev_status_texts[i].value = "Checking..."
                    self.dev_status_texts[i].color = "#999999"
                self.page.update()

                self.device_service.connect_serial(temp_port, temp_baud, gas_port, gas_baud)

                # 개별 장치 ping
                status = self.device_service.check_connections()

                self._update_device_indicator(0, status["temperature"])
                for i in range(4):
                    wo = status["gas"][i] and not self.device_service.gas_readable[i]
                    self._update_device_indicator(i + 1, status["gas"][i], write_only=wo)

                # 포트 열기 실패 표시
                if not temp_port:
                    self.dev_status_texts[0].value = "Port not set"
                    self.dev_status_texts[0].color = "#999999"
                    self.dev_icons[0].color = "#999999"
                elif not self.device_service.temp_client:
                    self.dev_status_texts[0].value = f"{temp_port} open failed"
                    self.dev_icons[0].color = "#F44336"

                if not gas_port:
                    for i in range(4):
                        self.dev_status_texts[i + 1].value = "Port not set"
                        self.dev_status_texts[i + 1].color = "#999999"
                        self.dev_icons[i + 1].color = "#999999"
                elif not self.device_service.gas_client:
                    for i in range(4):
                        self.dev_status_texts[i + 1].value = f"{gas_port} open failed"
                        self.dev_icons[i + 1].color = "#F44336"

                ok_count = (1 if status["temperature"] else 0) + sum(status["gas"])
                self.conn_status.value = f"Connected ({ok_count}/5 devices online)"
                self.conn_status.color = "#4CAF50" if ok_count == 5 else ("#FF9800" if ok_count > 0 else "red")

            self.page.update()

        except Exception as ex:
            self.conn_status.value = f"Connect failed: {ex}"
            self.conn_status.color = "red"
            for i in range(5):
                self._update_device_indicator(i, False)
            self.page.update()
  
    def _send_setpoint_from_settings(self, ch_idx: int):
        """세팅창에서 setpoint를 장치에 직접 전송"""
        try:
            val = float(self.sp_input_fields[ch_idx].value)
        except (ValueError, IndexError):
            self.sp_result_texts[ch_idx].value = "Invalid value"
            self.sp_result_texts[ch_idx].color = "#F44336"
            self.page.update()
            return

        try:
            self.device_service.set_gas(device_index=ch_idx, channel=1, value=val)
            self.sp_result_texts[ch_idx].value = f"Sent {val}"
            self.sp_result_texts[ch_idx].color = "#4CAF50"
        except Exception as ex:
            self.sp_result_texts[ch_idx].value = f"Failed: {ex}"
            self.sp_result_texts[ch_idx].color = "#F44336"

        self.page.update()

    def _apply_gas_channel(self, ch_idx: int):

        try:
            sp_field = self.channel_controls[ch_idx]["sp_field"]
            sp = float(sp_field.value) if sp_field and sp_field.value != "" else 0.0
        except Exception:
            sp = 0.0

        try:
            self.device_service.set_gas(
                device_index=ch_idx,
                channel=1,
                value=sp
            )

            self.conn_status.value = f"CH{ch_idx+1} -> {sp}"
            self.conn_status.color = "green"

        except Exception as ex:
            self.conn_status.value = f"Apply failed: {ex}"
            self.conn_status.color = "red"

        self.page.update()
  
    def _apply_all_configuration(self):
        # apply all channel setpoints to device
        try:
            for ci in range(4):
                try:
                    sp_field = self.channel_controls[ci]["sp_field"]
                    sp = float(sp_field.value) if sp_field and sp_field.value != "" else 0.0
                except Exception:
                    sp = 0.0
                # call existing apply for each channel
                self._apply_gas_channel(ci)
            try:
                self.page.update()
            except Exception:
                pass
        except Exception as e:
            print('apply_all_configuration error:', e)

    def _start_logging(self):
        # start trend/logging
        self.trend_running = True
        self.trend_run_button.text = "Stop"
        try:
            self.page.update()
        except Exception:
            pass

    def _refresh_status_table(self):

        if self.status_table is None:
            return

        rows = []

        try:
            # ✅ DeviceService 사용
            if hasattr(self, "device_service") and self.device_service:

                data = self.device_service.read_all()
                gas_list = data.get("gas", [])

                for i in range(4):

                    if i < len(gas_list) and gas_list[i] is not None:
                        g = gas_list[i]

                        gas_name = g.get("gas", "-")
                        sp = f"{g.get('sp', 0.0):.2f}"
                        flow = f"{g.get('pv', 0.0):.3f}"
                        press_val = g.get("pressure")
                        press = f"{press_val:.2f}" if press_val is not None else "-"

                    else:
                        gas_name = "-"
                        sp = "0.00"
                        flow = "0.00"
                        press = "-"

                    rows.append(
                        ft.DataRow(
                            cells=[
                                ft.DataCell(ft.Text(str(i + 1))),
                                ft.DataCell(ft.Text(gas_name)),
                                ft.DataCell(ft.Text(sp)),
                                ft.DataCell(ft.Text(flow)),
                                ft.DataCell(ft.Text(press)),
                            ]
                        )
                    )

            else:
                # 🔹 장치 연결 안된 경우 → UI 값 반영
                for i in range(4):
                    dd = self.channel_controls[i].get("gas_dd")
                    spf = self.channel_controls[i].get("sp_field")

                    gas = dd.value if dd else "-"
                    sp = (
                        f"{float(spf.value):.2f}"
                        if spf and spf.value != ""
                        else "0.00"
                    )

                    rows.append(
                        ft.DataRow(
                            cells=[
                                ft.DataCell(ft.Text(str(i + 1))),
                                ft.DataCell(ft.Text(gas)),
                                ft.DataCell(ft.Text(sp)),
                                ft.DataCell(ft.Text("0.00")),
                                ft.DataCell(ft.Text("-")),
                            ]
                        )
                    )

        except Exception as e:
            print("refresh_status_table error:", e)
            return

        # 테이블 교체
        try:
            self.status_table.rows.clear()
            self.status_table.rows.extend(rows)
            self.status_table.update()
        except Exception:
            pass
    
    def _start_gas_channel(self, ch_idx: int):

            self.schedule_mode = "gas"
            self.active_gas_channel = ch_idx
            self.schedule_time = 0.0

            # 해당 채널만 초기화
            targets = self._get_gas_targets(0.0)

            for i in range(4):
                if i == ch_idx:
                    init_val = targets[i]
                else:
                    init_val = 0.0

                self.history[i] = deque([init_val] * 600, maxlen=600)

            self.trend_running = True
            self.trend_run_button.text = "Stop"

            self.page.update()

    def _recompute_temp_rates(self):
        prev_temp = 25.0
        for i in range(8):
            rate_label = self.temp_steps[i]["rate_text"]
            if not self.temp_step_enabled[i]:
                rate_label.value = "-"
                continue
            tf = self.temp_steps[i]["temp_field"]
            df = self.temp_steps[i]["dur_field"]
            try:
                t_val = float(tf.value) if tf and tf.value != "" else prev_temp
            except Exception:
                t_val = prev_temp
            try:
                d_val = float(df.value) if df and df.value != "" else 0.0
            except Exception:
                d_val = 0.0

            if d_val > 0:
                rate = (t_val - prev_temp) / d_val
                rate_label.value = f"{rate:.1f}"
            else:
                rate_label.value = "-"

            prev_temp = t_val

    def _render_trend(self) -> str:
        """Render the full schedule as a dotted target line,
        and overlay the measured/simulated values that follow along.
        """
        fig, ax = plt.subplots(figsize=(10, 5.5), dpi=110)
        colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

        total_dur_hours = self._get_total_schedule_duration()
        total_dur_seconds = int(total_dur_hours * 3600) if total_dur_hours and total_dur_hours > 0 else 600
        total_dur_seconds = max(total_dur_seconds, 60)
        schedule_pts = min(total_dur_seconds, 3600)
        sched_times_h = [i * total_dur_hours / schedule_pts for i in range(schedule_pts + 1)]
        sched_times_s = [t * 3600.0 for t in sched_times_h]

        elapsed_seconds = int(round(self.schedule_time * 3600.0))

        if self.schedule_mode == 'temp':
            sched_targets = [self._get_schedule_target(t) for t in sched_times_h]
            ax.plot(sched_times_s, sched_targets, color=colors[1], linestyle='--',
                    linewidth=2, label="Schedule", alpha=0.7)

            n_measured = min(elapsed_seconds, len(self.measured))
            if n_measured > 1:
                measured_list = list(self.measured)[-n_measured:]
                meas_times = [elapsed_seconds - (n_measured - 1 - i) for i in range(n_measured)]
                ax.plot(meas_times, measured_list, color=colors[0], linewidth=2, label="Measured")

            ax.set_ylim(0, 100)
            ax.set_ylabel("Temperature (°C)", fontsize=13)

        else:
            for ch in range(4):
                ch_targets = [self._get_gas_targets(t)[ch] for t in sched_times_h]
                ax.plot(sched_times_s, ch_targets, color=colors[ch], linestyle='--',
                        linewidth=1.5, alpha=0.6)

            n_measured = min(elapsed_seconds, max(len(self.history[i]) for i in range(4)))
            if n_measured > 1:
                for ch in range(4):
                    ch_data = list(self.history[ch])[-n_measured:]
                    ch_times = [elapsed_seconds - (n_measured - 1 - i) for i in range(n_measured)]
                    ax.plot(ch_times, ch_data, color=colors[ch], linewidth=2, label=f"CH{ch+1}")

            ax.set_ylim(0, 15)
            ax.set_ylabel("Flow (sccm)", fontsize=13)

        ax.set_xlim(0, max(total_dur_seconds, 60))
        ax.set_xlabel("Time (s)", fontsize=13)
        ax.tick_params(axis='both', labelsize=12)
        ax.legend(loc="upper right", fontsize=11)
        ax.grid(True, linestyle="--", alpha=0.4)

        if elapsed_seconds > 0 and elapsed_seconds < total_dur_seconds:
            ax.axvline(x=elapsed_seconds, color='red', linestyle=':', linewidth=1, alpha=0.6)

        plt.tight_layout()
        buf = io.BytesIO()
        fig.savefig(buf, format="png")
        plt.close(fig)
        buf.seek(0)
        img_b64 = base64.b64encode(buf.read()).decode("ascii")
        return f"data:image/png;base64,{img_b64}"

    async def _trend_loop_async(self):
        while True:
            try:
                # 1️⃣ Trend가 실행 중일 때만 동작
                if self.trend_running:

                    # -------------------------------------------------
                    # 2️⃣ 장비 데이터 읽기 (있을 경우만)
                    # -------------------------------------------------
                    device_data = None

                    if hasattr(self, "device_service") and self.device_service:
                        try:
                            device_data = self.device_service.read_all()
                        except Exception as e:
                            print("device read error:", e)

                    if device_data and hasattr(self, "data_service") and self.data_service:
                        try:
                            self.data_service.update(device_data)
                        except Exception as e:
                            print("data_service update error:", e)

                    # -------------------------------------------------
                    # 3️⃣ 스케줄 시간 증가 (1초 = 1/3600 hour)
                    # -------------------------------------------------
                    self.schedule_time += 1.0 / 3600.0

                    total_dur = self._get_total_schedule_duration()
                    if total_dur > 0 and self.schedule_time >= total_dur:
                        self.schedule_time = total_dur
                        self.trend_image.src = self._render_trend()
                        self.trend_image.update()
                        self.trend_running = False
                        self.trend_run_button.text = "Start"
                        self.page.update()
                        continue

                    # -------------------------------------------------
                    # 4️⃣ 그래프 데이터 업데이트
                    # -------------------------------------------------
                    if self.schedule_mode == "temp":
                        # 장비값 우선, 없으면 스케줄 타겟
                        target = self._get_schedule_target(self.schedule_time)

                        last = self.measured[-1] if len(self.measured) else target

                        approach = 0.30      # 0.2~0.3 사이가 자연스러움
                        noise = random.uniform(-0.25, 0.25)

                        nv = last + (target - last) * approach + noise
                        nv = max(0.0, nv)

                        self.measured.append(nv)

                    else:  # gas mode

                        targets = self._get_gas_targets(self.schedule_time)

                        manual_mode = (self.control_mode == "Manual Mode")

                        for ch in range(4):

                            ch_target = targets[ch] if targets and ch < len(targets) else 0.0
                            last = self.history[ch][-1] if len(self.history[ch]) else ch_target

                            # Manual Mode일 때 비활성 채널은 값 유지
                            if manual_mode:
                                if self.active_gas_channel is not None and ch != self.active_gas_channel:
                                    self.history[ch].append(last)
                                    continue

                            approach = 0.3
                            noise = random.uniform(-0.02, 0.02)

                            nv = last + (ch_target - last) * approach + noise
                            nv = max(0.0, nv)

                            self.history[ch].append(nv)

                    # -------------------------------------------------
                    # 5️⃣ 그래프 다시 그리기
                    # -------------------------------------------------
                    self.trend_image.src = self._render_trend()
                    self.trend_image.update()

                    # 상태 테이블 갱신
                    try:
                        self._refresh_status_table()
                    except Exception:
                        pass

                    self.page.update()

                    await asyncio.sleep(1)

                else:
                    await asyncio.sleep(0.2)

            except Exception as ex:
                print("trend loop error:", ex)
                await asyncio.sleep(1)

    def _toggle_trend(self, e):
        self.trend_running = not self.trend_running
        self.trend_run_button.text = "Stop" if self.trend_running else "Start"
        # when starting, reset schedule time and measured buffer to begin at 0s
        if self.trend_running:
            self.schedule_time = 0.0

            if self.schedule_mode == 'temp':
                init_val = self._get_schedule_target(0.0)
                self.measured = deque([init_val] * 600, maxlen=600)
            else:
                targets = self._get_gas_targets(0.0)
                for i in range(4):
                    v = targets[i] if targets and i < len(targets) else 0.0
                    self.history[i] = deque([v] * 600, maxlen=600)

        try:
            self.page.update()
        except Exception:
            pass

    def _zoom_in(self, e):
        # increase window by 10s, up to 600s
        self.window_samples = min(600, self.window_samples + 10)
        self.zoom_label.value = f"Window: {self.window_samples}s"
        try:
            self.trend_image.src = self._render_trend()
            self.trend_image.update()
            self.page.update()
        except Exception:
            pass

    def _zoom_out(self, e):
        # decrease window by 10s, minimum 10s
        self.window_samples = max(10, self.window_samples - 10)
        self.zoom_label.value = f"Window: {self.window_samples}s"
        try:
            self.trend_image.src = self._render_trend()
            self.trend_image.update()
            self.page.update()
        except Exception:
            pass
    

    def _get_total_schedule_duration(self) -> float:
        """Return total schedule duration in hours based on current mode (enabled steps only)."""
        if self.schedule_mode == "temp":
            total = 0.0
            for i, s in enumerate(self.temp_steps):
                if not self.temp_step_enabled[i]:
                    continue
                df = s.get("dur_field")
                try:
                    total += float(df.value) if df and df.value != "" else 0.0
                except Exception:
                    pass
            return total
        else:
            if self.control_mode == "Mixing Mode":
                total = 0.0
                for i, slot in enumerate(self.mixing_steps):
                    if not self.mixing_step_enabled[i]:
                        continue
                    df = slot.get("dur_field")
                    try:
                        total += float(df.value) if df and df.value != "" else 0.0
                    except Exception:
                        pass
                return total
            else:
                max_total = 0.0
                for ch_idx, ch_steps in enumerate(self.gas_steps):
                    ch_total = 0.0
                    for i, slot in enumerate(ch_steps):
                        if not self.gas_step_enabled[ch_idx][i]:
                            continue
                        df = slot.get("dur_field")
                        try:
                            ch_total += float(df.value) if df and df.value != "" else 0.0
                        except Exception:
                            pass
                    if ch_total > max_total:
                        max_total = ch_total
                return max_total

    def _get_schedule_target(self, t_hours: float) -> float:
        """Compute the target value at time t_hours into the schedule.
        For temperature: ramp between previous step temp and step temp over step duration.
        For gas: use each channel's setpoint and duration similarly, but we take CH1 as representative.
        """
        if self.schedule_mode == "temp":
            steps = []
            for i, s in enumerate(self.temp_steps):
                if not self.temp_step_enabled[i]:
                    continue
                tf = s.get("temp_field")
                df = s.get("dur_field")
                try:
                    t_val = float(tf.value) if tf and tf.value != "" else None
                except Exception:
                    t_val = None
                try:
                    d_val = float(df.value) if df and df.value != "" else 0.0
                except Exception:
                    d_val = 0.0
                if t_val is None:
                    continue
                steps.append((t_val, d_val))

            if not steps:
                return self.measured[-1] if self.measured else 25.0

            # compute cumulative durations to find current step
            cum = 0.0
            prev_temp = steps[0][0]
            for temp, dur in steps:
                if dur <= 0:
                    # instantaneous jump
                    prev_temp = temp
                    continue
                if t_hours < cum + dur:
                    # inside this step: ramp from prev_temp to temp
                    t_in = t_hours - cum
                    frac = max(0.0, min(1.0, t_in / dur))
                    return prev_temp + (temp - prev_temp) * frac
                cum += dur
                prev_temp = temp

            # if beyond total, return last temp
            return steps[-1][0]

    def _get_gas_targets(self, t_hours: float):

        channel_targets = [0.0] * 4

        # -------------------------
        # Mixing Mode
        # -------------------------
        if self.control_mode == "Mixing Mode":

            steps = []

            for i, slot in enumerate(self.mixing_steps):
                if not self.mixing_step_enabled[i]:
                    continue
                sp = slot.get("setpoint")
                df = slot.get("dur_field")

                try:
                    s_val = float(sp.value) if sp and sp.value != "" else None
                except:
                    s_val = None

                try:
                    d_val = float(df.value) if df and df.value != "" else 0.0
                except:
                    d_val = 0.0

                if s_val is None:
                    continue

                steps.append((s_val, d_val))

            if not steps:
                return channel_targets

            # 보간 계산
            cum = 0.0
            prev_sp = steps[0][0]
            target = steps[-1][0]

            for spv, dur in steps:
                if dur <= 0:
                    prev_sp = spv
                    continue

                if t_hours < cum + dur:
                    frac = (t_hours - cum) / dur
                    frac = max(0.0, min(1.0, frac))
                    target = prev_sp + (spv - prev_sp) * frac
                    break

                cum += dur
                prev_sp = spv

            # 선택된 채널에만 적용
            for ch in range(4):
                if self.mixing_selected[ch]:
                    channel_targets[ch] = target

            return channel_targets

        # -------------------------
        # Manual Mode
        # -------------------------
        else:
            # 기존 채널별 스케줄 로직 사용
            ...
            channel_targets = []

            for ch_index, ch_steps in enumerate(self.gas_steps):
                steps = []

                for i, slot in enumerate(ch_steps):
                    if not self.gas_step_enabled[ch_index][i]:
                        continue
                    sp = slot.get("setpoint")
                    df = slot.get("dur_field")

                    try:
                        s_val = float(sp.value) if sp and sp.value != "" else None
                    except Exception:
                        s_val = None

                    try:
                        d_val = float(df.value) if df and df.value != "" else 0.0
                    except Exception:
                        d_val = 0.0

                    if s_val is None:
                        continue

                    steps.append((s_val, d_val))

                # ✅ 스케줄이 없으면 UI Setpoint 사용
                if not steps:
                    try:
                        ui_sp_field = self.channel_controls[ch_index]["sp_field"]
                        ui_sp = float(ui_sp_field.value) if ui_sp_field and ui_sp_field.value != "" else 0.0
                    except Exception:
                        ui_sp = 0.0

                    channel_targets.append(ui_sp)
                    continue

                # 스케줄 보간 처리
                cum = 0.0
                prev_sp = steps[0][0]
                ch_target = steps[-1][0]

                for spv, dur in steps:
                    if dur <= 0:
                        prev_sp = spv
                        continue

                    if t_hours < cum + dur:
                        t_in = t_hours - cum
                        frac = max(0.0, min(1.0, t_in / dur))
                        ch_target = prev_sp + (spv - prev_sp) * frac
                        break

                    cum += dur
                    prev_sp = spv

                channel_targets.append(ch_target)

            return channel_targets
    

def main(page: ft.Page):
    SchedulerApp(page)


if __name__ == "__main__":
    ft.run(main)
