# 4P 标定（Rust 工作区）常用入口
#
# 使用系统 FFmpeg/OpenCV 开发包；版本与可选覆盖变量见 docs/operations.md。
# 标准安装由绑定库自动发现，不固定本机路径或链接清单。

CARGO ?= cargo

.PHONY: help build test lint fmt check-deps gui gui-online camera-offline camera-live drills-mock drill-check clock-calibrate

help:
	@echo "make build          构建（默认只产出产线入口：rigcal-camera / rigcal-gui）"
	@echo "make test           全量测试"
	@echo "make lint           clippy（要求零告警）"
	@echo "make check-deps     检查两个产线入口的原生库版本与链接记录，不连接相机"
	@echo "make gui            四路仪表盘（配置里的引导源；示例配置是本地视频演练）"
	@echo "make gui-online     四路仪表盘走真机：相机 10.21.12.162，证据端点同机 30432"
	@echo "make camera-offline 单相机产线离线回放（--frames 目录）"
	@echo "make camera-live    单相机产线在线（配置里的 guidance + evidence）"
	@echo "make drill-check    现场链路自检：RTSP 四路 + raw 帧组（只读）"
	@echo "make clock-calibrate 只读：标定 RTSP↔raw 的时钟偏移与漂移率"
	@echo "make drills-mock    起证据帧 mock（Rust，端口 4211；需先备好 .pgm 帧目录 FRAMES=…）"

build:
	$(CARGO) build

check-deps:
	$(CARGO) run --locked -p rigcal-camera --bin rigcal-camera -- --check-deps
	$(CARGO) run --locked -p rigcal-gui -- --check-deps

test:
	$(CARGO) test

lint:
	$(CARGO) clippy --all-targets -- -D warnings

fmt:
	$(CARGO) fmt

gui:
	$(CARGO) run -p rigcal-gui -- --config crates/rigcal-gui/example.rig.yaml

gui-online:
	$(CARGO) run -p rigcal-gui -- --config crates/rigcal-gui/example.rig.yaml --rtsp-base 10.21.12.162

camera-offline:
	@test -n "$(FRAMES)" || (echo "用法: make camera-offline FRAMES=<图片目录> OUT=<输出目录>"; exit 2)
	$(CARGO) run -p rigcal-camera --bin rigcal-camera -- --config local/camera_session.yaml --frames "$(FRAMES)" --out "$(OUT)"

camera-live:
	$(CARGO) run -p rigcal-camera --bin rigcal-camera -- --config local/camera_session.yaml --live --out "$(OUT)"

drill-check:
	@echo "① raw 帧服务（板端端口以 ~/demo/config/sensor_config.yaml 的 raw_server.port 为准）"
	$(CARGO) run -p rigcal-io --features drills --bin raw-probe -- --host 10.21.12.162 --port 30432 --camera 0 --frames 3
	@echo "② 四路同刻帧组"
	$(CARGO) run -p rigcal-io --features drills --bin group-probe -- --host 10.21.12.162 --port 30432 --frames 3
	@echo "③ 四路 RTSP 引导"
	@for p in 554 555 556 557; do \
		$(CARGO) run -q -p rigcal-io --features drills --bin decode-probe -- --url "rtsp://10.21.12.162:$$p/PRR" --frames 2 --expect 1280x1088 | tail -1; \
	done

clock-calibrate:
	@echo "只读：标定 RTSP 帧时间戳 ↔ 板端 raw 时间戳的常数偏移与漂移率"
	$(CARGO) run -p rigcal-io --features drills --bin clock-calibrate -- \
		--url $(or $(URL),rtsp://10.21.12.162:554/PRR) --raw $(or $(RAW),10.21.12.162:30432) \
		--camera $(or $(CAM),0) --seconds $(or $(SECONDS),90)

drills-mock:
	@test -n "$(FRAMES)" || (echo "用法: make drills-mock FRAMES=<.pgm 帧目录> [PORT=4211] [FPS=60]"; exit 2)
	$(CARGO) run -p rigcal-io --features drills --bin mock-raw-server -- --frames "$(FRAMES)" --port $(or $(PORT),4211) --fps $(or $(FPS),60)
