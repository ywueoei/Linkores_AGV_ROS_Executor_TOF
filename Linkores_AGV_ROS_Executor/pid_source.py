import time
import math

class Fork_PID:
    def __init__(self, P=1.13, I=0.025, D=1.5):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear_up_data(0)
        self.clear_down_data(0)
        self.last_error = [0,0,0,0,0,0,0,0,0,]
        self.last_output = [0,0,0,0,0,0,0,0,0,]
        self.last_error2 = 0
        self.windup_guard_up = 1.2
        self.windup_guard_down = 1.2
        self.pre_sepoint_up = 0
        self.pre_sepoint_down = 0
        self.get_forkup_speed_255_test = 1
        self.get_forkdown_speed_255_test = 1
        self.pre_feedback_value = 0
        self.rcv_speed = 0
        self.feedback_value_list=[0,0,0,0]

        #self.maxdownspeed_y = -0.066
        #self.limitdownspeed_y = -0.04
        #self.maxdownspeed_x = -0.2
        #self.limitdownspeed_x = -0.15

        self.maxdownspeed_y = -0.066
        self.limitdownspeed_y = -0.04
        self.maxdownspeed_x = -0.3
        self.limitdownspeed_x = -0.15

        self.down_k1=(self.maxdownspeed_y-self.limitdownspeed_y)/(self.maxdownspeed_x-self.limitdownspeed_x)
        self.down_b1=self.maxdownspeed_y-self.down_k1*self.maxdownspeed_x
        # print("FPID#down_k1 down_b1 ",self.down_k1,self.down_b1)
        
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

    def clear_up_data(self,conut_round_afterTargetpointChange_up):
        self.PTerm_up = 0.0
        self.ITerm_up = 0.0
        self.DTerm_up = 0.0
        self.output = 0.0
        self.output_tmp1 = 0
        self.output_tmp2 = 0
        self.output_up_tmp1_count = 35
        self.conut_round_afterTargetpointChange_up = conut_round_afterTargetpointChange_up
        self.get_stuck_count_up = 0
        # print("FPID#up startpoint reset(clear)")
        self.delta_error_up=[-0.0002,-0.0002,-0.0002,-0.0002,-0.0002,-0.0002,-0.0002,-0.0002,]
        self.error_list_up = [0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,]
        self.attack_flag_up = False
    def clear_down_data(self,conut_round_afterTargetpointChange_down):
        self.PTerm_down = 0.0
        self.ITerm_down = 0.0
        self.DTerm_down = 0.0
        self.output = 0.0
        self.output_tmp1 = 0
        self.output_tmp2 = 0
        self.output_down_tmp1_count = 30
        self.conut_round_afterTargetpointChange_down = conut_round_afterTargetpointChange_down
        self.get_stuck_count_down = 0
        # print("FPID#down startpoint reset(clear)")
        self.delta_error_down=[0.0002,0.0002,0.0002,0.0002,0.0002,0.0002,0.0002,0.0002,]
        self.error_list_down = [-0.001,-0.001,-0.001,-0.001,-0.001,-0.001,-0.001,-0.001,]
        self.attack_flag_down = False
        
		
    def update_up(self,setPoint,feedback_value):
        #up or down change,setpoint change,clear(be careful ITerm)
        #清除下降状态的数据，若中途发生上升目标点更换，清除上次上升的数据，关键清除积分项
        self.clear_down_data(20)
        if self.pre_sepoint_up!=setPoint:
            self.clear_up_data(20)#连续上升点切换时提供初始的加速次数，以获得足够的力气上升
        feedback_value_org = feedback_value
        #计算当前输入值与上次的输入值的差，如果差0.15m（速度超过0.6m/s 0.12/(2z周期*0.125s)），则认为为突变值，将前一个值赋予当前
        #当前算法只能去除一个毛刺的输入
        d_feedback_value = abs(feedback_value-self.pre_feedback_value)
        if (d_feedback_value>0.15) and self.pre_feedback_value !=0:
            # print("FPID#up:feedbackERR,now:",round(feedback_value,4),"pre:",round(self.pre_feedback_value,4),"res:",\
                #   round(d_feedback_value,4))
            feedback_value = self.pre_feedback_value
            self.pre_feedback_value = 0
        else:
            self.pre_feedback_value = feedback_value


        #feedback_value_sum = 0
        #for i in range(len(self.feedback_value_list)):
            #feedback_value_sum = feedback_value_sum+self.feedback_value_list[i]
        #feedback_value_avr =feedback_value_sum/self.feedback_value_list.__len__()
        feedback_value_test =self.feedback_value_list[0] * 0.05 + self.feedback_value_list[1] * 0.1 \
            + self.feedback_value_list[2] * 0.15 + self.feedback_value_list[0] * 0.2 + feedback_value * 0.5
        if abs(self.rcv_speed)>0.05:
            #feedback_value = feedback_value_avr*0.5+feedback_value*0.5
            feedback_value =self.feedback_value_list[0] * 0.05 + self.feedback_value_list[1] * 0.1 \
            + self.feedback_value_list[2] * 0.15 + self.feedback_value_list[3] * 0.2 + feedback_value * 0.5
            #print("FPID#up:moving",self.rcv_speed," ,avr:", round(feedback_value_avr, 4), "now:", round(self.feedback_value_list[0], 4), "res:", \
            #round(feedback_value, 4))
        self.feedback_value_list = [feedback_value] + self.feedback_value_list[0:self.feedback_value_list.__len__() - 1]
        error = setPoint - feedback_value


        
        #get error average for limit the speed=0 when subterminal 
        #取一定长度的err计算平均值，当err平均值足够小，很接近目标点时，强制输出0
        self.error_list_up = [error]+self.error_list_up[0:self.error_list_up.__len__()-1]
        error_sum = 0
        for i in range(len(self.error_list_up)):
            error_sum = error_sum+self.error_list_up[i]
        error_avr =error_sum/self.error_list_up.__len__()  
        
        #get derror average for update the start point of first line
        #取一定长度的derr计算平均，用于给手动切换到自动时，重新赋初始速度        
        self.delta_error_up = [error -  self.last_error[0]] + self.delta_error_up[0:self.delta_error_up.__len__()-1]
        derror_sum = 0
        for i in range(len(self.delta_error_up)):
            derror_sum = derror_sum+self.delta_error_up[i]
        derror_avr =derror_sum/self.delta_error_up.__len__()  
                
        self.PTerm_up = error
        #Separation integral 0.08m ，err=0.08m时开始计算积分
        if abs(error)<0.08:
            #矩形积分
            #self.ITerm_up = self.ITerm_up+error
            #梯形积分
            self.ITerm_up = self.ITerm_up + (error+self.last_error[0])/2
        else:
            self.ITerm_up = 0
        #limit I data    
        if (self.ITerm_up < -self.windup_guard_up):
            self.ITerm_up = -self.windup_guard_up
        elif (self.ITerm_up > self.windup_guard_up):
            self.ITerm_up = self.windup_guard_up
            
        self.DTerm_up = error - self.last_error[0]
        #print("PTerm_up ITerm_up DTerm_up",self.PTerm_up,self.ITerm_up,self.DTerm_up)
        
        self.last_error = [error] + self.last_error[0:self.last_error.__len__()-1]
        
        
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        speed = -self.DTerm_up/delta_time
      
        #first line第一条曲线
        #if abs(delta_error)<0.04:

        # print("FPID#upScount Tcount", self.output_up_tmp1_count, self.conut_round_afterTargetpointChange_up)
        self.output_tmp1 = 0.0005*self.output_up_tmp1_count
        self.output_up_tmp1_count = self.output_up_tmp1_count+8
        if self.conut_round_afterTargetpointChange_up:
            self.conut_round_afterTargetpointChange_up = self.conut_round_afterTargetpointChange_up - 1
            if self.conut_round_afterTargetpointChange_up==0:
                self.attack_flag_up = False

        if abs(self.PTerm_up)>0.07 and abs(self.rcv_speed)<0.02 :
            if abs(derror_avr) < 0.005:#m
                self.get_stuck_count_up = self.get_stuck_count_up+1
                # print("FPID#up Try attack count ", self.get_stuck_count_up)
                if abs(derror_avr) < 0.0002 and self.conut_round_afterTargetpointChange_up==0 :
                    self.output_up_tmp1_count = 70
                    self.conut_round_afterTargetpointChange_up = 2
                    # print("FPID#up startpoint reset(Davr<0.0002)")

                if self.get_stuck_count_up >8*60:# and self.conut_round_afterTargetpointChange_up==0:
                    self.output_up_tmp1_count = 50
                    self.conut_round_afterTargetpointChange_up = 24
                    self.get_stuck_count_up = 0
                    self.attack_flag_up = True
                    # print("FPID#up Try attack ###")
            else:
                self.get_stuck_count_up = 0
                self.attack_flag_up = False
        else:
            self.get_stuck_count_up = 0

        #second line第二条曲线
        self.output_tmp2 = self.Kp*self.PTerm_up + (self.Ki * self.ITerm_up) + (self.Kd * self.DTerm_up)
        #print("PID",self.output, self.Kp,self.PTerm_up , self.Ki , self.ITerm_up, self.Kd , self.DTerm_up)
        
        #choose which line for output
        if self.output_tmp1 > self.output_tmp2:
            self.output = self.output_tmp2
            # print("kPID use ------------------------------------2",self.output_tmp2)
        else:
            self.output = self.output_tmp1
            # print("kPID use ------------------------------------1",self.output_tmp1)
            self.ITerm_up = 0

        #很接近目标时，强制输出为0
        if abs(error_avr)<0.005:
            self.output = 0
            # print("FPID#up:Pavr<0.005,set 0")

        if self.output<0:
            self.output = 0
            # print("FPID#up:speed<0 ERR,set 0")
        #self.output = self.output_tmp2
        #print("derror ",self.delta_error_up,"self.output_tmp1", self.output_tmp1,self.output_down_tmp1_count,self.output,error_avr)
        self.pre_sepoint_up=setPoint
        
        self.last_time = self.current_time
        return self.output

    def update_down2(self,setPoint,feedback_value):
        self.clear_up_data(20)#下降切换到上升，点切换时提供初始的加速次数，以获得足够的力气上升
        if self.pre_sepoint_down!=setPoint:
            self.clear_down_data(20)
        feedback_value_org = feedback_value
        #计算当前输入值与上次的输入值的差，如果差0.15m（速度超过0.6m/s 0.12/(2z周期*0.125s)），则认为为突变值，将前一个值赋予当前
        # 当前算法只能去除一个毛刺的输入
        d_feedback_value = abs(feedback_value-self.pre_feedback_value)
        if (d_feedback_value>0.15) and self.pre_feedback_value !=0:
            # print("FPID#down:feedbackERR,now:",round(feedback_value,4),"pre:",round(self.pre_feedback_value,4),\
            #       "res:",round(d_feedback_value,4))
            feedback_value = self.pre_feedback_value
            self.pre_feedback_value = 0
        else:
            self.pre_feedback_value = feedback_value


        #feedback_value_sum = 0
        #for i in range(len(self.feedback_value_list)):
            #feedback_value_sum = feedback_value_sum+self.feedback_value_list[i]
        #feedback_value_avr =feedback_value_sum/self.feedback_value_list.__len__()
        feedback_value_test = self.feedback_value_list[0] * 0.05+self.feedback_value_list[1] * 0.1\
                             +self.feedback_value_list[2] * 0.15+self.feedback_value_list[0] * 0.2+ feedback_value * 0.5
        if abs(self.rcv_speed)>0.05:
            #feedback_value = feedback_value_avr*0.5+feedback_value*0.5
            feedback_value = self.feedback_value_list[0] * 0.05+self.feedback_value_list[1] * 0.1\
                             +self.feedback_value_list[2] * 0.15+self.feedback_value_list[0] * 0.2+ feedback_value * 0.5
            #print("FPID#down:moving",self.rcv_speed," ,avr:", round(feedback_value_avr, 4), "now:", round(self.feedback_value_list[0], 4), "res:", \
            #round(feedback_value, 4))

        self.feedback_value_list = [feedback_value] + self.feedback_value_list[0:self.feedback_value_list.__len__() - 1]
        error = setPoint - feedback_value
        
        self.error_list_down = [error]+self.error_list_down[0:self.error_list_down.__len__()-1]
        error_sum = 0
        for i in range(len(self.error_list_down)):
            error_sum = error_sum+self.error_list_down[i]
        error_avr =error_sum/self.error_list_down.__len__()
                  
        self.delta_error_down = [error - self.last_error2] + self.delta_error_down[0:self.delta_error_down.__len__()-1]
        derror_sum = 0
        for i in range(len(self.delta_error_down)):
            derror_sum = derror_sum+self.delta_error_down[i]
        derror_avr =derror_sum/self.delta_error_down.__len__()    
            
        self.PTerm_down = error#比例

        if abs(error)<0.1:
            #矩形积分
            #self.ITerm_down = self.ITerm_down+error#积分
            #梯形积分
            self.ITerm_down = self.ITerm_down + (error+self.last_error2)/2
        else:
            self.ITerm_down = 0

        #self.ITerm_down = self.ITerm_down+error#积分
        if (self.ITerm_down < -self.windup_guard_down):
            self.ITerm_down = -self.windup_guard_down
        elif (self.ITerm_down > self.windup_guard_down):
            self.ITerm_down = self.windup_guard_down
        self.DTerm_down = error - self.last_error2
        #print("PTerm_down ITerm_down DTerm_down",self.PTerm_down,self.ITerm_down,self.DTerm_down)

        # print("FPID#downScount Tcount", self.output_down_tmp1_count, self.conut_round_afterTargetpointChange_down)
        #if delta_error<0.04:
        self.output_tmp1 = -0.0005*self.output_down_tmp1_count
        self.output_down_tmp1_count = self.output_down_tmp1_count+8
        if self.conut_round_afterTargetpointChange_down:
            self.conut_round_afterTargetpointChange_down = self.conut_round_afterTargetpointChange_down - 1
            if self.conut_round_afterTargetpointChange_down==0:
                self.attack_flag_down = False

        if abs(self.PTerm_down)>0.09 and abs(self.rcv_speed)<0.02 :
            if abs(derror_avr) < 0.005:#m
                self.get_stuck_count_down = self.get_stuck_count_down+1
                # print("FPID#down Try attack count ", self.get_stuck_count_down)
                if abs(derror_avr) < 0.0002 and self.conut_round_afterTargetpointChange_down==0 :
                    self.output_down_tmp1_count = 70
                    self.conut_round_afterTargetpointChange_down = 2
                    # print("FPID#down startpoint reset(Davr<0.0002)")

                if self.get_stuck_count_down >8*60 :#and self.conut_round_afterTargetpointChange_down==0:
                    self.output_down_tmp1_count = 50
                    self.conut_round_afterTargetpointChange_down = 24
                    self.get_stuck_count_down = 0
                    self.attack_flag_down = True
                    # print("FPID#down Try attack ###")
            else:
                self.get_stuck_count_down = 0
                self.attack_flag_down = False

        else:
            self.get_stuck_count_down = 0
        #self.output = self.output_tmp1
        
        
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        speed = -self.DTerm_down/delta_time
        

        #self.output_tmp2 = self.Kp * self.PTerm_down + (self.Ki * self.ITerm_down) + (self.Kd * self.DTerm_down)
        #if (self.PTerm_down<self.limitdownspeed_x):
        if (self.PTerm_down<self.limitdownspeed_x):
            output_tmp3= self.down_k1*self.PTerm_down+self.down_b1
        else:
            output_tmp3 = self.limitdownspeed_y

        output_tmp4 = self.Kp*self.PTerm_down + (self.Ki * self.ITerm_down) + (self.Kd * self.DTerm_down)

        if(output_tmp3<output_tmp4):
            self.output_tmp2 = output_tmp4
            # print("kPID use ------------------------------------4",output_tmp4)
        else:
            self.output_tmp2 = output_tmp3
            # print("kPID use ------------------------------------3",output_tmp3)
        
        #note output_tmp1 output_tmp2 is negative
        if self.output_tmp1 < self.output_tmp2 and self.attack_flag_down==False:
            self.output = self.output_tmp2
            # print("FPID22:TNAOC",round(setPoint,4),round(feedback_value_org,4),round(feedback_value,4),\
            #     round(self.output,4),round(self.output/self.get_forkdown_speed_255_test*255),"PID",round(self.PTerm_down,5),round(self.ITerm_down,5),round(self.DTerm_down,5)
            #     ,"avrPD",round(error_avr,4),round(derror_avr,4))

        else:
            self.output = self.output_tmp1
            # print("FPID21:TNAOC",round(setPoint,4),round(feedback_value_org,4),round(feedback_value,4),\
            #     round(self.output,4),round(self.output/self.get_forkdown_speed_255_test*255),"PID",round(self.PTerm_down,5),round(self.ITerm_down,5),round(self.DTerm_down,5)
            #     ,"avrPD",round(error_avr,4),round(derror_avr,4))

            self.ITerm_down = 0

        #self.output = self.output_tmp2
                              
        #limit the down speed,Otherwise it's not easy to calculate.限制最大下降速度
        if self.output<self.maxdownspeed_y:
        #if speed<-0.12:
            self.output = self.maxdownspeed_y
            # print("FPID#down:limit max speed ",self.output,round(self.output/self.get_forkdown_speed_255_test*255))
            
        if abs(error_avr)<0.005:
            self.output = 0
            # print("FPID#down:Pavr>-0.005,set 0")

        if self.output>0:
            self.output = 0
            # print("FPID#down:speed>0 ERR,set 0")
        self.last_error2 = error
        self.pre_sepoint_down = setPoint
        #print("derror ",self.delta_error_down,"self.output_tmp1", self.output_tmp1,self.output_down_tmp1_count,self.output,error_avr)
        
        # print("FPID:dT speed Out Can",round(speed,4),round(self.output,4),round(self.output/self.get_forkdown_speed_255_test*255),round(speed/self.output,4) if self.output!=0 else 0 )
        self.last_time = self.current_time
        return self.output
    def setKp(self, proportional_gain):
        self.Kp = proportional_gain
    def setKi(self, integral_gain):
        self.Ki = integral_gain
    def setKd(self, derivative_gain):
        self.Kd = derivative_gain
    def setWindup(self, windup):
        self.windup_guard_down = windup
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
    def set_testdata(self,forkup_speed_255_test,forkdown_speed_255_test):
        self.get_forkup_speed_255_test = forkup_speed_255_test
        self.get_forkdown_speed_255_test = forkdown_speed_255_test

    def set_carstatus(self,rcv_speed):
        self.rcv_speed = rcv_speed