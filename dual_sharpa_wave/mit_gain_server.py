"""Serialized gain service; uses the hand node's existing callback group."""

from sharpa_control_interfaces.srv import MitGains

from .mit_gains import GainTransition, PERIOD


class MitGainServer:
    def __init__(self, node, backend, clock):
        self.transition = GainTransition(backend)
        self.timer = node.create_timer(PERIOD, self.step, clock=clock)
        self.timer.cancel()
        self.service = node.create_service(MitGains, 'mit_gains', self.request)

    def request(self, request, response):
        response.success = True
        try:
            if request.operation == MitGains.Request.READ:
                self.transition.read()
            elif request.operation == MitGains.Request.APPLY:
                self.transition.apply(request.kp, request.kd)
                if self.transition.remaining:
                    self.timer.reset()
                else:
                    self.timer.cancel()
            elif request.operation == MitGains.Request.CANCEL:
                self.transition.cancel()
                self.timer.cancel()
            elif request.operation != MitGains.Request.STATUS:
                raise ValueError('Unknown MIT gains operation')
            response.success = not self.transition.failed
            response.message = self.transition.message
        except Exception as error:
            response.success = False
            response.message = str(error)
        response.kp, response.kd = self.transition.kp, self.transition.kd
        response.remaining_steps = self.transition.remaining
        return response

    def step(self):
        # Reset before the synchronous operation: no burst of queued timer ticks.
        self.timer.reset()
        self.transition.step()
        if not self.transition.remaining:
            self.timer.cancel()
