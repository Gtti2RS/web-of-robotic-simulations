import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

# Interfaces
from sim_process_supervisor_interfaces.srv import (
    ExecCmd,          # string cmd -> int32 return_code, string stdout, string stderr
    ManagedStart,     # string name, string cmd -> bool success, string message
    ManagedStop,      # string name -> bool success, string message
    ManagedStatus,    # string name -> bool running, int32 pid, string message
    ManagedList,      #  -> string[] names, int32[] pids, bool[] running
)


class ProcessSupervisor(Node):
    def __init__(self):
        super().__init__('process_supervisor')

        # Parameters
        self.declare_parameter('grace_timeout_sec', 5)
        self.declare_parameter('log_child_output', True)

        # --- Locks & state ---
        self._proc_lock = threading.RLock()    # reentrant to avoid self-deadlocks

        # Managed multi-process: name -> {'proc', 'out_t', 'err_t'}
        self._procs = {}

        # --- Services (exec one-shot returning output) ---
        self.srv_exec_cmd = self.create_service(ExecCmd, 'process/exec',  self.handle_exec_cmd)

        # --- Services (managed, multi-process by name) ---
        self.srv_mstart  = self.create_service(ManagedStart,  'process/managed/start',  self.handle_managed_start)
        self.srv_mstop   = self.create_service(ManagedStop,   'process/managed/stop',   self.handle_managed_stop)
        self.srv_mstatus = self.create_service(ManagedStatus, 'process/managed/status', self.handle_managed_status)
        self.srv_mlist   = self.create_service(ManagedList,   'process/managed/list',   self.handle_managed_list)

        self.get_logger().info('ProcessSupervisor ready.')

    # -------------------- Utility: shared readers --------------------

    def _reader(self, pipe, tag):
        """Continuously read a pipe and log each line."""
        try:
            for line in iter(pipe.readline, ''):
                if not line:
                    break
                self.get_logger().info(f'[{tag}] {line.rstrip()}')
        except Exception as e:
            self.get_logger().warn(f'[{tag}] reader error: {e}')
        finally:
            try:
                pipe.close()
            except Exception:
                pass

    def _attach_readers(self, proc, tag_prefix: str):
        """Attach stdout/stderr reader threads if enabled."""
        if not bool(self.get_parameter('log_child_output').value):
            return None, None
        out_t = threading.Thread(target=self._reader, args=(proc.stdout, f'{tag_prefix}:stdout'), daemon=True)
        err_t = threading.Thread(target=self._reader, args=(proc.stderr, f'{tag_prefix}:stderr'), daemon=True)
        out_t.start()
        err_t.start()
        return out_t, err_t

    # -------------------- Managed (multi-child) helpers --------------------

    def _is_running_entry(self, entry):
        p = (entry or {}).get('proc')
        return p is not None and p.poll() is None

    def _spawn_managed(self, name: str, cmdline: str):
        with self._proc_lock:
            entry = self._procs.get(name)
            if entry and self._is_running_entry(entry):
                return False, f'[{name}] already running (PID {entry["proc"].pid})'
            try:
                proc = subprocess.Popen(
                    ['bash', '-lc', cmdline],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid,
                    text=True,
                    bufsize=1
                )
                out_t, err_t = self._attach_readers(proc, name)
                self._procs[name] = {'proc': proc, 'out_t': out_t, 'err_t': err_t}
                return True, f'[{name}] started PID {proc.pid}'
            except Exception as e:
                return False, f'[{name}] failed to start: {e}'

    def _stop_managed(self, name: str, grace: float):
        with self._proc_lock:
            entry = self._procs.get(name)
            if not entry or not self._is_running_entry(entry):
                return False, f'[{name}] not running'
            proc = entry['proc']
            pid = proc.pid
            try:
                pgid = os.getpgid(pid)
            except Exception:
                pgid = pid

        # signals outside lock
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return True, f'[{name}] already stopped'

        t0 = time.time()
        while time.time() - t0 < grace:
            if proc.poll() is not None:
                return True, f'[{name}] stopped (SIGINT)'
            time.sleep(0.1)

        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            return True, f'[{name}] stopped (SIGTERM attempted)'

        t1 = time.time()
        while time.time() - t1 < 2.0:
            if proc.poll() is not None:
                return True, f'[{name}] stopped (SIGTERM)'
            time.sleep(0.1)

        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass

        try:
            proc.wait(timeout=1.0)
        except Exception:
            pass
        return True, f'[{name}] killed (SIGKILL)'

    # -------------------- Service handlers --------------------

    # Exec one-shot and return output
    def handle_exec_cmd(self, req: ExecCmd.Request, res: ExecCmd.Response):
        raw = (req.cmd or '').strip()
        self.get_logger().info(f'/process/exec called with cmd: {raw!r}')
        if not raw:
            res.return_code = -1
            res.stdout = ''
            res.stderr = 'Empty command'
            return res
        try:
            proc = subprocess.run(
                ['bash', '-lc', raw],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            res.return_code = int(proc.returncode)
            res.stdout = proc.stdout or ''
            res.stderr = proc.stderr or ''
            self.get_logger().info(f'/process/exec done (rc={proc.returncode})')
            return res
        except Exception as e:
            res.return_code = -1
            res.stdout = ''
            res.stderr = str(e)
            self.get_logger().error(f'/process/exec failed: {e}')
            return res

    # Managed multi-process by name
    def handle_managed_start(self, req: ManagedStart.Request, res: ManagedStart.Response):
        name = (req.name or '').strip()
        cmd  = (req.cmd or '').strip()
        if not name or not cmd:
            res.success, res.message = False, 'name and cmd required'
            return res
        ok, msg = self._spawn_managed(name, cmd)
        self.get_logger().info(msg)
        res.success, res.message = ok, msg
        return res

    def handle_managed_stop(self, req: ManagedStop.Request, res: ManagedStop.Response):
        name = (req.name or '').strip()
        if not name:
            res.success, res.message = False, 'name required'
            return res
        grace = float(self.get_parameter('grace_timeout_sec').value)
        ok, msg = self._stop_managed(name, grace)
        self.get_logger().info(msg)
        res.success, res.message = ok, msg
        return res

    def handle_managed_status(self, req: ManagedStatus.Request, res: ManagedStatus.Response):
        name = (req.name or '').strip()
        with self._proc_lock:
            entry = self._procs.get(name)
            running = self._is_running_entry(entry) if entry else False
            pid = entry['proc'].pid if (entry and running) else -1
        res.running = bool(running)
        res.pid = int(pid)
        res.message = f'[{name}] RUNNING (PID {pid})' if running else f'[{name}] NOT RUNNING'
        return res

    def handle_managed_list(self, req: ManagedList.Request, res: ManagedList.Response):
        names, pids, running = [], [], []
        with self._proc_lock:
            for name, entry in self._procs.items():
                r = self._is_running_entry(entry)
                names.append(name)
                pids.append(entry['proc'].pid if (entry and r) else -1)
                running.append(bool(r))
        res.names = names
        res.pids = [int(x) for x in pids]
        res.running = [bool(x) for x in running]
        return res

    # -------------------- Lifecycle --------------------

    def _shutdown_all_managed(self, grace: float):
        with self._proc_lock:
            names = list(self._procs.keys())
        for n in names:
            try:
                ok, msg = self._stop_managed(n, grace)
                self.get_logger().info(f'Shutdown managed {n}: {msg}')
            except Exception as e:
                self.get_logger().warn(f'Error stopping managed {n}: {e}')

def main():
    rclpy.init()
    node = ProcessSupervisor()
    try:
        rclpy.spin(node)
    finally:
        # Stop all managed children
        try:
            node._shutdown_all_managed(float(node.get_parameter('grace_timeout_sec').value))
        except Exception as e:
            node.get_logger().warn(f'Error stopping managed children on shutdown: {e}')

        node.destroy_node()
        rclpy.shutdown()
