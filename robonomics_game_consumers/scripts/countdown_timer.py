import time
import threading

class CountdownTimer:
    def __init__(self, period=0, step=1, hook=None):
        self.period = period
        self.step = step
        self.remaining = 0
        self.hook = hook

    def start(self, period=None, hook=None):
        self.period = period or self.period
        self.hook = hook or self.hook
        threading.Thread(target=self._cd, args=(self.period, self.hook)).start()

    def _cd(self, period, hook):
        self.remaining = period
        while self.remaining > 0.0:
            time.sleep(self.step)
            self.remaining -= self.step
        self.remaining = 0
        if hook:
            hook()
