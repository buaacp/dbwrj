import threading
from typing import Optional


class PlannerWorker:
    def __init__(self, planner):
        self.planner = planner
        self._thread = None
        self._result = None
        self._error = None

    def start_plan(self, *args, **kwargs) -> None:
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("planner is already running")
        self._result = None
        self._error = None

        def run():
            try:
                self._result = self.planner.plan(*args, **kwargs)
            except Exception as exc:
                self._error = exc

        self._thread = threading.Thread(target=run)
        self._thread.daemon = True
        self._thread.start()

    def done(self) -> bool:
        return self._thread is not None and not self._thread.is_alive()

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result
