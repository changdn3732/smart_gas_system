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

from simulators.alicat_controller import AlicatController
from devices.device_service import DeviceService
from data.data_service import DataService

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
    
    def _on_gas_type_change(self, ch: int, gas: str):
        self.channel_gases[ch] = gas
        print(f"CH{ch+1} Gas changed to {gas}")

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
        # initialize 8 temp steps
        self.temp_steps = [
            {"temp_field": None, "dur_field": None, "rate_text": None} for _ in range(8)
        ]
        # gas control state
        self.control_mode = "Mixing Mode"
        self.selected_gas_channel = 0
        self.active_gas_channel = None
        # build UI
        self.schedule_panel = self._build_schedule_panel()
        self.available_gases = [
            "N2",
            "O2",
            "Ar",
            "H2",
            "He",
            "Air"
        ]

        self.channel_gases = ["N2", "N2", "N2", "N2"]
        # Prepare in-memory history for 4 channels (keep up to 600 samples)
        self.history = {i: deque([0.0] * 600, maxlen=600) for i in range(4)}
        # single measured series that will approximate the schedule (keep up to 600 samples)
        self.measured = deque([25.0] * 600, maxlen=600)
        # gas steps storage: for 4 channels, each has up to 8 steps
        self.gas_steps = [
            [{"setpoint": None, "dur_field": None, "rate_text": None} for _ in range(8)] for _ in range(4)
        ]
        # Mixing mode용 공통 스케줄
        self.mixing_steps = [
            {"setpoint": None, "dur_field": None} for _ in range(8)
        ]

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

        # layout: left 60% (expand=6), right 40% (expand=4)
        layout = ft.Row([
            ft.Container(content=self.schedule_panel, expand=6, padding=12),
            ft.Container(width=20),
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
            ]), expand=4, padding=12),
        ], expand=True)

        page.add(layout)

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

    def _render_schedule_content(self):
        self.schedule_content.controls.clear()
        if self.schedule_mode == "temp":
            self.schedule_content.controls.append(ft.Text("Temperature Scheduling", size=14, weight=ft.FontWeight.BOLD))
            rows = []
            for i in range(8):
                step_idx = i + 1
                if not self.temp_steps[i]["temp_field"]:
                    tf = ft.TextField(label=f"Step {step_idx} Temp (°C)", width=140, value="25.0",
                                      on_change=lambda e, idx=i: self._on_temp_or_dur_change(idx))
                    df = ft.TextField(label="Duration (hours)", width=140, value="0.5",
                                      on_change=lambda e, idx=i: self._on_temp_or_dur_change(idx))
                    rate = ft.Text("Rate: -", size=11)
                    self.temp_steps[i]["temp_field"] = tf
                    self.temp_steps[i]["dur_field"] = df
                    self.temp_steps[i]["rate_text"] = rate
                else:
                    tf = self.temp_steps[i]["temp_field"]
                    df = self.temp_steps[i]["dur_field"]
                    rate = self.temp_steps[i]["rate_text"]

                rows.append(ft.Row([
                    ft.Text(f"Step {step_idx}", width=60),
                    tf,
                    ft.Container(width=8),
                    df,
                    ft.Container(width=8),
                    rate,
                ], alignment=ft.MainAxisAlignment.START))

            self.schedule_content.controls.extend(rows)
            self._recompute_temp_rates()
            btn_apply = self._make_sq_button("Apply Temp Schedule", on_click=lambda e: print("Apply temp schedule"))
            btn_start = self._make_sq_button("Start Schedule", on_click=lambda e: print("Start temp schedule"))
            self.schedule_content.controls.append(ft.Row([
                btn_apply.control,
                ft.Container(width=12),
                btn_start.control,
            ]))

        else:
            # -----------------------------
            # 🔝 Gas Control Mode
            # -----------------------------
            self.schedule_content.controls.append(
                ft.Text("Gas Control Mode", size=14, weight=ft.FontWeight.BOLD)
            )

            mode_row = ft.Row([
                self._make_sq_button(
                    "Mixing Mode",
                    on_click=lambda e: self._set_control_mode("Mixing Mode"),
                    bgcolor="#003366" if self.control_mode == "Mixing Mode" else "#cccccc"
                ).control,
                ft.Container(width=10),
                self._make_sq_button(
                    "Manual Mode",
                    on_click=lambda e: self._set_control_mode("Manual Mode"),
                    bgcolor="#003366" if self.control_mode == "Manual Mode" else "#cccccc"
                ).control,
            ])

            self.schedule_content.controls.append(mode_row)
            self.schedule_content.controls.append(ft.Divider())

            # ==========================================================
            # 🔷 MIXING MODE
            # ==========================================================
            if self.control_mode == "Mixing Mode":

                # 🔘 다중 선택 버튼
                ch_row = ft.Row()

                for ch in range(4):
                    selected = self.mixing_selected[ch]

                    btn = self._make_sq_button(
                        f"CH{ch+1}",
                        on_click=lambda e, c=ch: self._toggle_mixing_channel(c),
                        bgcolor="#003366" if selected else "#cccccc"
                    )

                    ch_row.controls.append(btn.control)
                    ch_row.controls.append(ft.Container(width=10))

                self.schedule_content.controls.append(ch_row)
                self.schedule_content.controls.append(ft.Divider())

                # 📋 공통 스케줄 표시
                self.schedule_content.controls.append(
                    ft.Text("Common Mixing Schedule (Step 1~8)", weight=ft.FontWeight.BOLD)
                )

                for step in range(8):

                    slot = self.mixing_steps[step]

                    if not slot["setpoint"]:
                        sp_field = ft.TextField(
                            label=f"Step {step+1} SP",
                            width=140,
                            value="0.0"
                        )

                        dur_field = ft.TextField(
                            label="Duration (h)",
                            width=140,
                            value="0.5"
                        )

                        slot["setpoint"] = sp_field
                        slot["dur_field"] = dur_field
                    else:
                        sp_field = slot["setpoint"]
                        dur_field = slot["dur_field"]

                    self.schedule_content.controls.append(
                        ft.Row([
                            ft.Text(f"S{step+1}", width=50),
                            sp_field,
                            ft.Container(width=10),
                            dur_field
                        ])
                    )

            # ==========================================================
            # 🔷 MANUAL MODE
            # ==========================================================
            else:

                # 🔘 단일 선택 버튼
                ch_row = ft.Row()

                for ch in range(4):
                    btn = self._make_sq_button(
                        f"CH{ch+1}",
                        on_click=lambda e, c=ch: self._select_gas_schedule_channel(c),
                        bgcolor="#003366" if self.selected_gas_channel == ch else "#cccccc"
                    )

                    ch_row.controls.append(btn.control)
                    ch_row.controls.append(ft.Container(width=10))

                self.schedule_content.controls.append(ch_row)
                self.schedule_content.controls.append(ft.Divider())

                # 📋 선택 채널 스케줄 표시
                ch = self.selected_gas_channel

                self.schedule_content.controls.append(
                    ft.Text(f"CH{ch+1} Schedule (Step 1~8)", weight=ft.FontWeight.BOLD)
                )

                for step in range(8):

                    slot = self.gas_steps[ch][step]

                    if not slot["setpoint"]:
                        sp_field = ft.TextField(
                            label=f"Step {step+1} SP",
                            width=140,
                            value="0.0"
                        )

                        dur_field = ft.TextField(
                            label="Duration (h)",
                            width=140,
                            value="0.5"
                        )

                        slot["setpoint"] = sp_field
                        slot["dur_field"] = dur_field
                    else:
                        sp_field = slot["setpoint"]
                        dur_field = slot["dur_field"]

                    self.schedule_content.controls.append(
                        ft.Row([
                            ft.Text(f"S{step+1}", width=50),
                            sp_field,
                            ft.Container(width=10),
                            dur_field
                        ])
                    )               
    def _on_temp_or_dur_change(self, idx: int):
                    try:
                        self._recompute_temp_rates()
                        self.page.update()
                    except Exception:
                        pass




    def _open_settings(self, e):
        # create settings view if not present
        try:
            if not hasattr(self, "settings_view"):
                # device connection controls
                self.conn_status = ft.Text("Not connected", color="red")
                # device type selector: Simulator or Serial
                self.device_type = ft.Dropdown(options=[ft.dropdown.Option("Simulator"), ft.dropdown.Option("Serial")], value="Simulator", width=180)
                self.port_field = ft.TextField(label="Serial Port", value="COM3", width=160)
                self.baud_field = ft.TextField(label="Baudrate", value="9600", width=120)
                # device id mapping fields
                self.ut32a_id_field = ft.TextField(label="UT32A Slave ID", value="1", width=120)
                self.alicat_start_id_field = ft.TextField(label="Alicat Start ID", value="2", width=120)
                connect_btn = self._make_sq_button("Connect Device", on_click=lambda ev: self._toggle_connection())


                # =========================
                # 🔥 Gas Configuration Section
                # =========================

                gas_config_section = ft.Column(
                    [
                        ft.Text("Gas Configuration", size=14, weight=ft.FontWeight.BOLD),
                        ft.Divider(),
                    ],
                    spacing=8,
                )

                for ch in range(4):

                    dropdown = ft.Dropdown(
                        width=200,
                        value=self.channel_gases[ch],
                        options=[ft.dropdown.Option(g) for g in self.available_gases],
                    )

                    dropdown.on_change = lambda e, c=ch: self._on_gas_type_change(c, e.control.value)

                    gas_config_section.controls.append(
                        ft.Row([
                            ft.Text(f"CH{ch+1}", width=60),
                            dropdown
                        ])
                    )

                self.settings_view = ft.View(route="/settings", controls=[
                    ft.AppBar(title=ft.Text("Settings"), bgcolor="#ffffff", leading=ft.TextButton("Back", on_click=self._close_settings)),
                    ft.Container(content=ft.Column([
                        ft.Text("Device Connection", size=14, weight=ft.FontWeight.BOLD),
                        ft.Row([ft.Text("Device Type:"), self.device_type, ft.Container(width=12), ft.Text("Port:"), self.port_field, ft.Container(width=12), ft.Text("Baud:"), self.baud_field]),
                        ft.Row([ft.Text("UT32A ID:"), self.ut32a_id_field, ft.Container(width=12), ft.Text("Alicat Start ID:"), self.alicat_start_id_field]),
                        self.conn_status,
                        connect_btn.control,
                        ft.Divider(height=12),
                        gas_config_section,
                        ft.Divider(height=12),
                        ft.Text("Connection Status:", size=12),
                        ft.Text("Placeholder: All devices nominal", color="#080"),
                    ], spacing=12), padding=12)
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

    def _toggle_connection(self, e=None):
        # attempt to connect/disconnect based on selected device type
        try:
            if getattr(self, 'device', None) is None:
                # connect
                    dtype = self.device_type.value if hasattr(self, 'device_type') else 'Simulator'
                    # read id mappings
                    try:
                        ut32a_id = int(self.ut32a_id_field.value) if hasattr(self, 'ut32a_id_field') else 1
                    except Exception:
                        ut32a_id = 1
                    try:
                        alicat_start = int(self.alicat_start_id_field.value) if hasattr(self, 'alicat_start_id_field') else 2
                    except Exception:
                        alicat_start = 2
                    # store mapping
                    self.device_config = {"ut32a_id": ut32a_id, "alicat_start_id": alicat_start}
                    if dtype == 'Simulator':
                        self.device_service = DeviceService()
                        self.device_service.connect_simulator()
                        self.data_service = DataService()
                        self.conn_status.value = f"Connected (Simulator)"
                        self.conn_status.color = 'green'
                    else:
                        # Serial
                        port = self.port_field.value if hasattr(self, 'port_field') else None

                        try:
                            baud = int(self.baud_field.value) if hasattr(self, 'baud_field') else 9600
                        except Exception:
                            baud = 9600

                        try:
                            self.device_service = DeviceService()
                            self.device_service.connect_serial(port, baud)   # 🔥 핵심

                            self.conn_status.value = f"Connected ({port}@{baud})"
                            self.conn_status.color = 'green'

                        except Exception as ex:
                            self.conn_status.value = f"Connect failed: {ex}"
                            self.conn_status.color = 'red'
            else:
                # disconnect
                try:
                    if isinstance(self.device, AlicatController):
                        # simulator: just drop reference
                        del self.device
                    else:
                        # serial
                        try:
                            self.device.close()
                        except Exception:
                            pass
                        del self.device
                    self.conn_status.value = 'Not connected'
                    self.conn_status.color = 'red'
                except Exception:
                    pass

            try:
                self.page.update()
            except Exception:
                pass
        except Exception as e:
            print('toggle_connection error:', e)

    def _apply_gas_channel(self, ch_idx: int):
        try:
            sp_field = self.channel_controls[ch_idx]["sp_field"]
            sp = float(sp_field.value) if sp_field and sp_field.value != "" else 0.0
        except Exception:
            sp = 0.0

       
        print(f"Apply gas schedule to CH{ch_idx+1}: setpoint={sp}")
        try:
            # simulator path
            if getattr(self, 'device', None) is not None and isinstance(self.device, AlicatController):
                # Alicat simulator channels are 1-based
                dev_ch = ch_idx + 1
                try:
                    self.device.set_flow(dev_ch, sp)
                    self.conn_status.value = f"Applied CH{ch_idx+1} -> {sp} (sim)"
                    self.conn_status.color = 'green'
                except Exception as ex:
                    self.conn_status.value = f"Apply failed: {ex}"
                    self.conn_status.color = 'red'

            # serial path (basic line protocol placeholder)
            elif getattr(self, 'device', None) is not None and serial is not None and hasattr(self.device, 'write'):
                base = self.device_config.get('alicat_start_id', 2) if hasattr(self, 'device_config') else 2
                slave = base + ch_idx
                cmd = f"ID {slave} SET {sp}\n"
                try:
                    self.device.write(cmd.encode('ascii'))
                    self.conn_status.value = f"Sent to ID {slave}: {sp}"
                    self.conn_status.color = 'green'
                except Exception as ex:
                    self.conn_status.value = f"Send failed: {ex}"
                    self.conn_status.color = 'red'
            else:
                self.conn_status.value = 'No device connected'
                self.conn_status.color = 'red'

            try:
                self.page.update()
            except Exception:
                pass
        except Exception as e:
            print('apply_gas_channel error:', e)

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

    def _emergency_stop(self):
        try:
            if getattr(self, 'device', None) is not None and isinstance(self.device, AlicatController):
                self.device.emergency_stop()
                self.conn_status.value = 'Emergency stop sent (sim)'
                self.conn_status.color = 'red'
            elif getattr(self, 'device', None) is not None and hasattr(self.device, 'write'):
                # placeholder serial emergency command
                try:
                    self.device.write(b"EMERGENCY\n")
                    self.conn_status.value = 'Emergency sent (serial)'
                    self.conn_status.color = 'red'
                except Exception as ex:
                    self.conn_status.value = f'EMERGENCY send failed: {ex}'
                    self.conn_status.color = 'red'
            else:
                self.conn_status.value = 'No device connected'
                self.conn_status.color = 'red'
            try:
                self._refresh_status_table()
            except Exception:
                pass
            try:
                self.page.update()
            except Exception:
                pass
        except Exception as e:
            print('emergency_stop error:', e)

    def _refresh_status_table(self):
        # update the live status table from device (simulator) or keep placeholders
        if self.status_table is None:
            return
        rows = []
        try:
            if getattr(self, 'device', None) is not None and isinstance(self.device, AlicatController):
                st = self.device.get_data()
                for i in range(1, 5):
                    d = st.get(i, {})
                    gas = d.get('gas', '-')
                    sp = f"{d.get('sp', 0.0):.2f}"
                    flow = f"{d.get('flow', 0.0):.3f}"
                    press = f"{d.get('pressure', '-'):.2f}" if d.get('pressure') is not None else '-'
                    rows.append(ft.DataRow(cells=[ft.DataCell(ft.Text(str(i))), ft.DataCell(ft.Text(gas)), ft.DataCell(ft.Text(sp)), ft.DataCell(ft.Text(flow)), ft.DataCell(ft.Text(press))]))
            else:
                # no device: reflect UI fields
                for i in range(4):
                    dd = self.channel_controls[i].get('gas_dd')
                    spf = self.channel_controls[i].get('sp_field')
                    gas = dd.value if dd else '-'
                    sp = f"{float(spf.value):.2f}" if spf and spf.value != '' else '0.00'
                    rows.append(ft.DataRow(cells=[ft.DataCell(ft.Text(str(i+1))), ft.DataCell(ft.Text(gas)), ft.DataCell(ft.Text(sp)), ft.DataCell(ft.Text('0.00')), ft.DataCell(ft.Text('-'))]))
        except Exception as e:
            print('refresh_status_table error:', e)
            return

        # replace table rows
        try:
            self.status_table.rows.clear()
            self.status_table.rows.extend(rows)
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
            tf = self.temp_steps[i]["temp_field"]
            df = self.temp_steps[i]["dur_field"]
            rate_label = self.temp_steps[i]["rate_text"]
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
                rate_label.value = f"Rate: {rate:.2f} °/h"
            else:
                rate_label.value = "Rate: -"

            prev_temp = t_val

    def _render_trend(self) -> str:
        """Render current history to PNG and return base64 data URI.
        If the configured window covers the full schedule duration, render from t=0..T.
        Otherwise render the latest window_samples ending at current schedule_time.
        """
        fig, ax = plt.subplots(figsize=(12, 6), dpi=100)
        colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

        # total schedule duration in seconds
        total_dur_hours = self._get_total_schedule_duration()
        total_dur_seconds = int(total_dur_hours * 3600) if total_dur_hours and total_dur_hours > 0 else 0

        if self.schedule_mode == 'temp':
            # temperature-mode plotting (single measured series + target)
            if total_dur_seconds > 0 and self.window_samples >= total_dur_seconds:
                # show entire schedule from t=0 to total_dur_seconds (cap to buffer size)
                samples = min(total_dur_seconds, self.measured.maxlen)
                sample_times_hours = [i / 3600.0 for i in range(samples)]
                sample_times_seconds = [i for i in range(samples)]

                # compute target across schedule
                targets = [self._get_schedule_target(t) for t in sample_times_hours]

                # measured values may only exist for recent times; align measured deque to absolute times
                measured_plot = [math.nan] * samples
                measured_len = len(self.measured)
                if measured_len > 0:
                    measured_start_time = self.schedule_time - (measured_len - 1) / 3600.0
                    measured_list = list(self.measured)
                    for idx in range(samples):
                        t = sample_times_hours[idx]
                        if measured_start_time <= t <= self.schedule_time:
                            sec_offset = int(round((t - measured_start_time) * 3600.0))
                            if 0 <= sec_offset < measured_len:
                                measured_plot[idx] = measured_list[sec_offset]

                ax.plot(sample_times_seconds, measured_plot, label="Measured", color=colors[0])
                ax.plot(sample_times_seconds, targets, label="Target", color=colors[1], linestyle="--")
            else:
                # recent-window view: last N samples ending at schedule_time
                n = max(1, min(self.window_samples, len(self.measured)))
                data = list(self.measured)[-n:]
                # sample times in seconds since schedule start
                sample_times_seconds = [int(round(self.schedule_time * 3600.0)) - (n - 1 - i) for i in range(n)]
                targets = []
                sample_times_hours = []
                for i in range(n):
                    sample_time_h = self.schedule_time - (n - 1 - i) / 3600.0
                    sample_times_hours.append(sample_time_h)
                    targets.append(self._get_schedule_target(sample_time_h))

                ax.plot(sample_times_seconds, data, label="Measured", color=colors[0])
                ax.plot(sample_times_seconds, targets, label="Target", color=colors[1], linestyle="--")

        else:
            # gas-mode plotting (per-channel histories + targets)
            # compute visible window
            n = max(1, min(self.window_samples, max(len(self.history[i]) for i in range(4))))
            # sample times in seconds
            sample_times_seconds = [int(round(self.schedule_time * 3600.0)) - (n - 1 - i) for i in range(n)]
            sample_times_hours = [self.schedule_time - (n - 1 - i) / 3600.0 for i in range(n)]

            # plot each channel
            for ch in range(4):
                data = list(self.history[ch])[-n:]
                ax.plot(sample_times_seconds, data, label=f"CH{ch+1}", color=colors[ch])

            # plot channel targets as dashed lines
            targets_per_channel = [self._get_gas_targets(t) for t in sample_times_hours]
            # transpose targets_per_channel to per-channel lists
            for ch in range(4):
                ch_targets = [tp[ch] if tp and ch < len(tp) and tp[ch] is not None else math.nan for tp in targets_per_channel]
                ax.plot(sample_times_seconds, ch_targets, color=colors[ch], linestyle='--', linewidth=1)

        ax.legend(loc="upper right", fontsize=8)
        ax.set_ylabel("Value")
        ax.set_xlabel("Seconds")
        # ensure x-axis starts at 0 when schedule begins
        try:
            if 'sample_times_seconds' in locals() and len(sample_times_seconds) > 0:
                xmax = max(sample_times_seconds)
                ax.set_xlim(0, xmax)

            # Y축 고정 범위 설정
            if self.schedule_mode == "temp":
                ax.set_ylim(0, 100)
            else:
                ax.set_ylim(0, 15)
        except Exception:
            pass
        ax.grid(True, linestyle="--", alpha=0.4)
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
                        self.schedule_time = 0.0

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

                        manual_mode = (
                            hasattr(self, "control_mode_dd")
                            and self.control_mode_dd
                            and self.control_mode_dd.value == "Manual Mode"
                        )

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
        """Return total schedule duration in hours based on current mode."""
        if self.schedule_mode == "temp":
            total = 0.0
            for s in self.temp_steps:
                df = s.get("dur_field")
                try:
                    total += float(df.value) if df and df.value != "" else 0.0
                except Exception:
                    pass
            return total
        else:
            # compute the total duration as the maximum total among gas channels
            max_total = 0.0
            for ch_steps in self.gas_steps:
                ch_total = 0.0
                for slot in ch_steps:
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
            # build step list of (temp, dur)
            steps = []
            for i, s in enumerate(self.temp_steps):
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

            for slot in self.mixing_steps:
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

                for slot in ch_steps:
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
