#!/bin/bash
# ============================================================
# Go1 FSM 统一启停脚本
#   ./run_fsm.sh start    启动 + 日志记录
#   ./run_fsm.sh stop     停止
#   ./run_fsm.sh restart  重启
#   ./run_fsm.sh status   查看状态
#   ./run_fsm.sh log      实时查看日志
# ============================================================

set -e

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
BINARY="$BUILD_DIR/go1_fsm"
PID_FILE="/tmp/go1_fsm.pid"
LOG_DIR="$PROJECT_DIR/logs"
LOG_FILE="$LOG_DIR/fsm_$(date +%Y%m%d_%H%M%S).log"
LATEST_LOG="$LOG_DIR/latest.log"

# 确保日志目录存在
mkdir -p "$LOG_DIR"

# ============================================================
do_start() {
    if [ -f "$PID_FILE" ] && kill -0 $(cat "$PID_FILE") 2>/dev/null; then
        echo "[FSM] Already running (PID: $(cat $PID_FILE))"
        return 1
    fi

    if [ ! -x "$BINARY" ]; then
        echo "[FSM] Binary not found, building..."
        cd "$BUILD_DIR" && make -j$(nproc) go1_fsm
    fi

    echo "[FSM] Starting... (log: $LOG_FILE)"
    cd "$BUILD_DIR"
    nohup stdbuf -oL ./go1_fsm >> "$LOG_FILE" 2>&1 &
    echo $! > "$PID_FILE"
    ln -sf "$LOG_FILE" "$LATEST_LOG"
    sleep 1

    if kill -0 $(cat "$PID_FILE") 2>/dev/null; then
        echo "[FSM] Started (PID: $(cat $PID_FILE))"
        echo "[FSM] Log: tail -f $LATEST_LOG"
    else
        echo "[FSM] Failed to start! Check $LOG_FILE"
        rm -f "$PID_FILE"
        return 1
    fi
}

do_stop() {
    if [ ! -f "$PID_FILE" ]; then
        echo "[FSM] Not running (no PID file)"
        return 0
    fi

    PID=$(cat "$PID_FILE")
    if kill -0 $PID 2>/dev/null; then
        echo "[FSM] Stopping (PID: $PID)..."
        kill -TERM $PID
        for i in $(seq 1 30); do
            kill -0 $PID 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL $PID 2>/dev/null || true
        echo "[FSM] Stopped."
    else
        echo "[FSM] Process $PID not found, cleaning PID file."
    fi
    rm -f "$PID_FILE"
}

do_status() {
    if [ -f "$PID_FILE" ] && kill -0 $(cat "$PID_FILE") 2>/dev/null; then
        PID=$(cat "$PID_FILE")
        echo "[FSM] Running (PID: $PID)"
        echo "[FSM] Log: $LATEST_LOG"
        echo "[FSM] Last 5 lines:"
        tail -5 "$LATEST_LOG" 2>/dev/null | sed 's/^/  /'
    else
        echo "[FSM] Not running."
        rm -f "$PID_FILE"
    fi
}

do_log() {
    if [ -f "$LATEST_LOG" ]; then
        tail -f "$LATEST_LOG"
    else
        echo "[FSM] No log file yet."
    fi
}

# ============================================================
case "${1:-start}" in
    start)   do_start ;;
    stop)    do_stop ;;
    restart) do_stop; sleep 0.5; do_start ;;
    status)  do_status ;;
    log)     do_log ;;
    *)
        echo "Usage: $0 {start|stop|restart|status|log}"
        echo ""
        echo "  遥控器控制:"
        echo "    R2+A → Stand    R2+B → Damping"
        echo "    R2+X → Model    R2+Y → Down"
        exit 1
        ;;
esac
