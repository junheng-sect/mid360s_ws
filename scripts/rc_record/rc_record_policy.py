"""RC 开关触发录包的纯策略（不依赖 ROS、不做 IO）。

设计参照板端 orange 工作空间里 vz_step_control 的 offboard_bag_recorder：
把「何时可以开包 / 何时该关包 / 记录器是否健康」的决策从 rospy 节点里拆出来，
这样策略可以用脚本化的假时钟确定性地跑测试。

与本项目那份实现的差异：
  * 触发源是 RC 通道电平（AUX 开关），不是 OFFBOARD 上升沿；
  * 增加「去抖（confirm_samples）」与「松手后保留一段尾巴（release_tail_s）」；
  * 与飞控 ulog 的配对判据同向：PX4 logger 在 SDLOG_MODE=3 下用
    manual_control_setpoint.aux1 > 0.3 决定是否记录，而 aux1 由 PWM 归一化
    （1000->-1, 1500->0, 2000->+1），即 PWM 约 >1650us 才开始录 ulog。
    所以本策略默认 active_high（PWM >= 阈值 视为触发），包与 ulog 同起同停。
  * 只有先看到过「有效且未触发」的电平，才允许开包：节点被重启/respawn 时
    若开关已经在触发位，不会凭空生成一个半截包。
"""


class RcRecordSession(object):
    """RC 触发录包的状态机。

    调用方（节点）负责真正的开/关包与序号分配，并通过 bag_opened() /
    bag_closed() / fail() 把结果回灌进来。
    """

    OPEN = 'open'
    PENDING_CLOSE = 'pending_close'
    CLOSE = 'close'

    def __init__(self, confirm_samples=3, release_tail_s=1.0):
        if confirm_samples < 1:
            raise ValueError('confirm_samples must be >= 1')
        if release_tail_s < 0.0:
            raise ValueError('release_tail_s must be >= 0')
        self.confirm_samples = int(confirm_samples)
        self.release_tail_s = float(release_tail_s)

        self._rc_valid = False
        self._rc_lost = True
        self._raw_active = None
        self._pending_count = 0
        self._active = False
        self._armed = False
        self._bag_open = False
        self._fatal = False
        self._close_at = None

    def on_rc(self, now, valid, active):
        """喂一次采样；返回 OPEN / PENDING_CLOSE / CLOSE / None。"""
        if not valid:
            # RC 丢失：不改动触发电平（正在录的包不能因丢链路被截断），
            # 只清掉去抖计数，等重新收到数据重新计数。
            self._rc_valid = False
            self._rc_lost = True
            self._raw_active = None
            self._pending_count = 0
            return None

        self._rc_valid = True
        self._rc_lost = False
        active = bool(active)
        if not active:
            self._armed = True

        if self._raw_active is None or active != self._raw_active:
            self._raw_active = active
            self._pending_count = 1
        else:
            self._pending_count += 1

        if self._pending_count < self.confirm_samples:
            return None

        if active and not self._active:
            self._active = True
            self._close_at = None          # 尾巴期内重新触发 -> 同一个包继续录
            if self._armed and not self._fatal and not self._bag_open:
                return self.OPEN
            return None

        if (not active) and self._active:
            self._active = False
            if self._bag_open:
                if self.release_tail_s <= 0.0:
                    return self.CLOSE
                self._close_at = now + self.release_tail_s
                return self.PENDING_CLOSE
        return None

    def tick(self, now):
        """与 RC 数据无关的心跳（例如 5 Hz 定时器）。

        必须由调用方周期驱动，否则 RC 链路断了、松手时，尾巴到期没人关包。
        """
        if self._close_at is not None and now >= self._close_at and self._bag_open:
            self._close_at = None
            return self.CLOSE
        return None

    def note_rc_loss(self):
        """RC 丢失时的外部动作（stop_on_rc_loss=true 时节点把它当作松手）。"""
        if self._bag_open and self._close_at is None:
            return self.PENDING_CLOSE if self.release_tail_s > 0 else self.CLOSE
        return None

    def bag_opened(self):
        self._bag_open = True

    def bag_closed(self):
        self._bag_open = False

    def fail(self):
        self._fatal = True
        self._bag_open = False

    @property
    def active(self):
        return self._active

    @property
    def rc_valid(self):
        return self._rc_valid

    @property
    def rc_lost(self):
        return self._rc_lost

    @property
    def close_due_at(self):
        return self._close_at

    def recording(self):
        return self._bag_open

    def healthy(self):
        return not self._fatal

    def __repr__(self):
        return ('RcRecordSession(active={}, bag={}, armed={}, fatal={}, '
                'pending={})').format(self._active, self._bag_open, self._armed,
                                      self._fatal, self._pending_count)
