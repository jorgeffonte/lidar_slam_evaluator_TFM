import numpy as np
import rosbag


class TimeComp:
    def __init__(self, file_name):
        """Get time comp data from a file

        Args:
            file_name (string): data's file name 
        """
        print("Reading {}".format(file_name))
        self.is_None = False
        
        if file_name.endswith('.bag'):
            with rosbag.Bag(file_name) as bag:
                comp_time,pub_time,name = self._read_bag(bag)
        else:
            print("unsupported type of data file")
            self.is_None = True

        
        self.comp_time_list = np.array(comp_time)
        self.pub_time_list = np.array(pub_time)
        self.name = name



    def _read_bag(self, bag):
        comp_time_list = []
        pub_time_list = []
        t0=None
        for topic, msg, t in bag.read_messages():
            if t0==None:
                t0 = t
            comp_time_list.append(msg.data)
            pub_time_list.append((t-t0).to_sec())  # Doesnt have header.stamp, so we use the time of the message

        return comp_time_list, pub_time_list,topic

  
