import can
import cantools
from threading import Thread
import os 

class motor_driver():
    def __init__(self, cobid = 0, interface='socketcan', channel='can0', bitrate=500000, dicfile = 'can_parser.dbc', ger_box_ratio = 6.0):
        self.cobid = cobid
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = can.ThreadSafeBus(interface= self.interface, channel=self.channel, bitrate=self.bitrate)
        self.dicfile = dicfile
        self.can_db = cantools.database.load_file( dicfile)
        self._base_can_frames = {'motor_config_1' : 1, 'motor_config_2' : 2, 'motor_config_3' : 3, 'motor_out_elec': 4,  
                                 'motor_out_mec' : 5, 'transition': 6, 'motor_sp' : 7} #can frames function  dictionary
        self._is_on = True
        self.motor_elec_out ={'Ua': 0.0, 'Ub': 0.0, 'current': 0.0, 'electrical_angle': 0.0}
        self.motor_mec_out ={'shaft_angle': 0.0, 'shaft_angle_sp': 0.0, 'shaft_velocity': 0.0}
        self._transition_table = [[0, 2, 0, 0, 0, 0, 0],[0, 0, 1, 3, 0, 0, 0],
                                  [0, 0, 0, 0, 2, 4, 0], [0, 0, 0, 0, 0, 0, 2]]
        self._states_table = [[1 , 2, 0, 0],[1, 2, 3, 4],
                             [0, 2, 3, 4], [1, 0, 0, 4]]
        self._motor_states ={'init': 1, 'preoperational': 2, 'running': 3, 'stop': 4} #Internal Finite State Machine State
        self._current_state = 1 
        self._gearbox_ratio = ger_box_ratio
        self._motor_position = 0.0
        self._read_bus = Thread(target = self.read_bus) #Thread to poll the bus while the motor is running
        self._read_bus.start()
        
    def change_state(self, transition):
        #If the transition exists for the current state in the transition table, change state
        if 1 <= transition <len(self._transition_table[0]):
            if self._transition_table[self._current_state - 1][transition] != 0:
                self._current_state = self._transition_table[self._current_state - 1][transition]
                self.send_msg(self._base_can_frames['motor_sp'] + self.cobid , [transition])
                return True
            else:
                print('Incorrect transition')
                return False
        else:
            return False
             
    def send_config_1(self, P_controller = 2.0, I_controller = 0.01, D_controller = 0):
        real_frame = self.cobid + self._base_can_frames['motor_config_1']
        canConfig1 = self.can_db.encode_message(self._base_can_frames['motor_config_1'],{'P_control' : P_controller,
                                                                                         'I_control': I_controller,  'D_control' : D_controller})
        ack = self.send_msg(real_frame ,canConfig1)
        return ack

    def send_config_2(self, P_vel = 2.0, I_vel = 0.01, D_vel = 0, Vel_lim  = 20.0):
        real_frame = self.cobid + self._base_can_frames['motor_config_2']
        canConfig2 = self.can_db.encode_message(self._base_can_frames['motor_config_2'],{'P_vel' : P_vel, 
                                                                                         'I_vel': I_vel,  'D_vel' : D_vel,  'Vel_lim' : Vel_lim})
        ack = self.send_msg(real_frame ,canConfig2)
        return ack
        
    def send_config_3(self, Volt_lim = 20.0, V_aling = 7.0, Calibrate = 0, Zero_angle_elec  = 0.0):
        real_frame = self.cobid + self._base_can_frames['motor_config_3']
        canConfig3 = self.can_db.encode_message(self._base_can_frames['motor_config_3'],{'Volt_lim' : Volt_lim,
                                                                                         'V_aling': V_aling,   'Calibrate' : Calibrate,  'Zero_angle_elec' : Zero_angle_elec})
        ack = self.send_msg(real_frame ,canConfig3)
        return ack   
    
    def send_transition(self, transition = 0):
        ack = self.change_state(transition = transition)
        if ack == True:
            real_frame = self.cobid + self._base_can_frames['transition']
            transition_msg = self.can_db.encode_message(self._base_can_frames['transition'],{'Transition' : transition})
            self.send_msg(real_frame ,transition_msg)
        else:
            print("Incorrect transition")
        return ack
            
    def send_command(self, angle):
        
        if  self._current_state == self._motor_states['running']: #If the motor is is running state, send the angle command
            angle = angle*self._gearbox_ratio
            encoded_message = self.can_db.encode_message(self._base_can_frames['motor_sp'],{'angle_sp' : angle})
            self.send_msg(self._base_can_frames['motor_sp'] + self.cobid , encoded_message)
        else:
            print('Only send commands on running mode!')
            
    def get_motor_angle(self):
        return self._motor_mec_out["shaft_angle"]/self._gearbox_ratio
    
    def read_bus(self):
        while self._is_on:
            msg = self.bus.recv(1)
            if msg is not None:
                    try:
                        if (msg.arbitration_id-self.cobid) == self._base_can_frames["motor_out_elec"]:
                            print('rec motor elec')
                            self._motor_elec_out = self.can_db.decode_message((msg.arbitration_id-self.cobid), msg.data)
                        elif (msg.arbitration_id-self.cobid) == self._base_can_frames["motor_out_mec"]:
                            self._motor_mec_out = self.can_db.decode_message((msg.arbitration_id-self.cobid), msg.data)
                    
                    except:
                        print("Error reading can message")
                    

    def send_msg(self,frame,msg):
        msg = can.Message(arbitration_id=frame, data=msg, is_extended_id=False)
        try:
            self.bus.send(msg)
            return True
        except (ValueError, TypeError) as e:
            print("Wrong CAN message data: " +  str(e))
            return False

        except can.CanError as e:
            print(" Failed to send CAN message: " + str(e))
            return False

    def shutdown(self):
        self._is_on = False
        self._read_bus.join()
        self.bus.shutdown()


