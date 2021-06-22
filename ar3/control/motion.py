import logging
import math

clamp = lambda n, abs_bound: max(min(abs_bound, n), -abs_bound)

# Represents PID state for a single joint
class PIDJoint:

    def __init__(self, **kwargs):
        self.motion_write_hz = 10000
        self.pid = [0.35, 0.5, 0.4] 
        self.max_spd = 5000
        self.max_accel = 40 
        self.initial_spd = 10
        self.vel_dead_zone = 10
        self.pos_dead_zone = 10
        self.max_pid_contribution = 10000
        self.__dict__.update(kwargs)

        self.step_vel = 0
        self.ticks_per_step = 0
        self.prev_pos = 0
        self.prev_vel = 0
        self.prev_err_vel = 0
        self.active = False
        self.err_vel = 0
        self.err_pos = 0
        self.pid_updates = [0,0,0]

    def dbg(self):
        print("====debug PIDJoint====")
        for k,v in self.__dict__.items():
            if type(v) == int or type(v) == float:
                print(k, v)

    # Velocities are implemented by slowly adjusting 
    # the stepping period for each joint based on the target
    # velocity and (currently hardcoded) acceleration profile given for each motor
    # dt is in seconds
    def compute_steps(self, actual_pos, actual_vel, intent_pos, intent_vel, dt):
        # Update kinematic state
        self.prev_err_vel = self.err_vel
        self.err_vel = intent_vel - actual_vel
        self.err_pos = intent_pos - actual_pos

        # Don't calculate stepping if we're already at intent
        self.active = (abs(self.err_vel) > self.vel_dead_zone) or (abs(self.err_pos) > self.pos_dead_zone)
        if not self.active:
            return 0
        
        # This is PID adjustment targeting velocity - in this case, P=velocity, I=position, D=acceleration
        # Note that we have targets both for position and velocity, but not for acceleration - that's where
        # we use a real value.
        self.pid_updates = [
                self.pid[0] * self.err_vel,
                self.pid[1] * self.err_pos, 
                self.pid[2] * (self.err_vel - self.prev_err_vel)]
        if max(self.pid_updates) > self.max_pid_contribution:
          raise Exception("PID update {self.pid_updates} contains at least 1 term above the hard limit {self.max_pid_contribution}")
        vel_adjust = clamp(sum(self.pid_updates), self.max_accel / dt)

        # Update velocity, applying velocity limits
        self.step_vel = clamp(self.step_vel + vel_adjust, self.max_spd)
      
        # If step_vel is too small, we overflow due to small divisor 
        if abs(self.step_vel) < self.initial_spd:
          self.step_vel = copysign(self.initial_spd, self.step_vel)

        # ticks/step = (ticks/sec) * (sec / step)
        #            = (ticks/sec) * (1 / step_speed)
        self.ticks_per_step = int(self.motion_write_hz / self.step_vel)
        # TODO bounds check, ensure within int32_t range
        return self.ticks_per_step

class MotionTriBotVel:
    NUM_J = 3

    def __init__(self, max_speed = 800):
        self.max_speed = max_speed
        self.j = [PIDJoint() for j in range(self.NUM_J)]

    def _unpack(self, ser_cmd):
        raise Exception("not implemented")

    def wheel_speed(self, vx, vy, theta):
        if vx == 0 and vy == 0:                                
          return 0            
        # Max 1.0 magnitude to prevent going faster than max_speed on a diagonal (which could be up to 1.414)                 
        mag = min(math.sqrt(vx*vx + vy*vy), 1.0)               
        ang = math.atan2(vy, vx)      
        return mag * math.cos(ang - theta)                                                                                    
    
    def _wheel_speeds(self, vx, vy, vr):                       
        if vx == 0 and vy == 0 and vr == 0:
            return None                                                 
        # Forward is -Y +Z
        # Left is +X,                                          
        spds = [self.wheel_speed(vx, vy, theta) for theta in [0, 2*math.pi/3, 4*math.pi/3]]                                             
        # Turning is added on top of existing speed            
        return [(s + vr) * self.max_speed for s in spds]

    def update(self, actual_pos, actual_vel, vx, vy, vr, dt):
        spds = self._wheel_speeds(vx, vy, vr)
        stepping = [self.j[i].compute_steps(actual_pos[i], actual_vel[i], spds[i], 0, dt) for i in range(self.NUM_J)]
        return stepping

    def debug_directions(self):                                
        # Simple check that the wheel magnitudes are reasonable                                                               
        print(self._wheel_speeds(0, 0, 0, 0))
        print(self._wheel_speeds(1, 0, 0, 0))
        print(self._wheel_speeds(-1, 0, 0, 0))
        print(self._wheel_speeds(0, 1, 0, 0))
        print(self._wheel_speeds(0, -1, 0, 0))

if __name__ == "__main__":
    print("Test PIDJoint")
    j = PIDJoint()
    print(j.compute_steps(0,0,100,100,0.1))
    print(j.dbg())

    print("Test MotionTriBotVel")
    m = MotionTriBotVel()
    print(m.update([0,0,0], [0,0,0], 0.5, 0.5, 0.5, 0.1))
