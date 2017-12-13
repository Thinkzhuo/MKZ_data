#include "controller/lonpid.hpp"

longctrlstate LongCtrlState;

LongControl::LongControl()
{
}

LongControl::~LongControl()
{
    ofs.close();
    ifs.close();
}

bool LongControl::init(float ts_, float rate_)
{
  ts = ts_;
  rate = rate_;

  ts = 0.1;

  e_v = e_v_pre = e_v_pre2 = 0;
  e_v_sum = 0;

  u_v = du_v = u_v_pre= 0;


  long_ctrl_state = 0;
  last_output_gb = 0;

  string logfilepath = "/auto_ws/data/controller/debug_lonctrl_pid/";
  string current_date = get_current_date();
  ofs.open(logfilepath + current_date + "_lonctrl_pid_debug.txt");

  return true;
}

int LongControl::long_ctrl_state_trans(bool enabled, float v_car, float v_des, float last_output_gb, int long_ctrl_state)
{
    bool stopping_condition = ( v_car < 1.8/3.6 ) && ( v_des < 1e-1 );
    bool starting_condition  = ( v_car < 1e-1 ) && ( 1.8/3.6 < v_des );
    const float brake_threshold_to_pid = 0.1;
//    bool pid_ctrl_condition = ( 0.5 < v_des ) && ( 0.5 < v_car );

    if(!enabled)
    {
        long_ctrl_state = LongCtrlState.off;
    }else{
        if(long_ctrl_state == LongCtrlState.off)
        {
            if(enabled)
                long_ctrl_state = LongCtrlState.pid_ctrl;
        }
        else if(long_ctrl_state == LongCtrlState.pid_ctrl)
        {
            if(stopping_condition)
            {
                long_ctrl_state = LongCtrlState.stopping;
            }
        }
        else if(long_ctrl_state == LongCtrlState.stopping)
        {
            if(starting_condition)
            {
                long_ctrl_state = LongCtrlState.starting;
            }
        }
        else if(long_ctrl_state == LongCtrlState.starting)
        {
            if(stopping_condition)
            {
                long_ctrl_state = LongCtrlState.stopping;
            }
            else if (last_output_gb >= - brake_threshold_to_pid)
            {
                long_ctrl_state = LongCtrlState.pid_ctrl;
            }
        }
    }
    return long_ctrl_state;
}

float LongControl::set_pid_vel(float kp, float ti, float td) {
    kp_v = kp;
    ti_v = ti;
    td_v = td;
}

float LongControl::update(float v_car, float v_des, float rate)
{
    set_pid_vel(0.3*0.7, 2*1.8, 0.3*0.3);

  //ofs << "car_vel=" << vel << ", dvel=" << dvel;
      float dvel = v_des;
      float vel = v_car;
      e_v = dvel - vel;

      e_v_sum += e_v;
      const float e_v_sum_max = 5/3.6;
      if(e_v_sum > e_v_sum_max) {
          e_v_sum = e_v_sum_max;
      } else if(e_v_sum < -e_v_sum_max) {
          e_v_sum = -e_v_sum_max;
      }

      // pid
      float u_v_p = kp_v*e_v;
      float u_v_i = kp_v*ts/ti_v*e_v_sum;
      float u_v_d = kp_v*td_v/ts*(e_v - e_v_pre);
      u_v = u_v_p + u_v_i + u_v_d;

      // dpid
  //    float u_v_p = kp_v*(e_v-e_v_pre);
  //    float u_v_i = kp_v*ts/ti_v*e_v;
  //    float u_v_d = kp_v*td_v/ts*(e_v - 2*e_v_pre + e_v_pre2);
  //    du_v = u_v_p + u_v_i + u_v_d;
  //    u_v += du_v;

      if(e_v > 0 && u_v < 0) {
          u_v = u_v_p;
      } else if(e_v < 0 && u_v > 0) {
          u_v = u_v_p;
      }
      if(e_v < 0 && e_v > -1/3.6) {
        u_v = 0;
      }
      if(u_v > 0) {
          u_v = 5 * u_v;
      }

      if(u_v >= 0.6) {
          u_v = 0.6;
      }
      if(u_v < -0.8) {
          u_v = -0.8;
      }

      ofs<<v_des<<"\t"<<v_car<<"\t"<<e_v<<"\t"
        <<u_v_p<<"\t"<<u_v_i<<"\t"<<u_v_d<<"\t"
       <<endl;

      //    ofs << "e_v=" << e_v << ", p_v=" << u_v_p << ", i_v=" << u_v_i << ", d_v=" << u_v_d
  //        << ", u_v=" << u_v << endl;

      ROS_INFO("pid_vel: e_v=%f, e_v_pre=%f, e_v_sum=%f, p=%f, i=%f, d=%f, u=%f",
               e_v, e_v_pre, e_v_sum, u_v_p, u_v_i, u_v_d, u_v);

      //ofs << "e_v=" << e_v << ", e_v_pre=" << e_v_pre << ", e_v_sum=" << e_v_sum
          //<< ", u_v=" << u_v << ", u_v_p=" << u_v_p << ", u_v_i=" << u_v_i << ", u_v_d=" << u_v_d << endl;

      e_v_pre2 = e_v_pre;
      e_v_pre = e_v;
      u_v_pre = u_v;

      return u_v;
}
