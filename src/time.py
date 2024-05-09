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
                comp_time = self._read_bag(bag)
        else:
            print("unsupported type of data file")
            self.is_None = True

        
        self.comp_time_list = np.array(comp_time)



    def _read_bag(self, bag):
        comp_time_list = []
        for topic, msg, _ in bag.read_messages():
        
            comp_time_list.append(msg.data)
        return comp_time_list

  
