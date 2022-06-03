#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""   __Signature__

    » author:   Hakan GENCTURK
    » mail to:  hakan@dofrobotics.com
    » linkedin: https://www.linkedin.com/in/hakangencturk/

                                                               **
                                                          ,/(######(/,
                                                      ,(################(,
                                                 .*(########################(*.
                                             .(##############(.  .(##############(.            .............................
    *#####################(//,.         .*/#############(*.          .*(#############/*.      *##########################/.
    *###########################*       ,###########(,                    ,(##########(,      *########################,
    *############################(   .#(*.  ,/((/,                            ,/((/,  .*(#.   *#####################/.
    *#############################/  .######/.      ..,,,,,,,,,,,,,,.,,,....      .(######.   *###################,
    *######(///////////((##########.  #######(         .,*////////////*,.         (#######    *#######*
    *######,               .#######* .#######(     *,       ,//////,       ,*     (#######.   *#######*
    *######,                *######* .#######(      ,//*,.            .,*//,      (#######.   *#######*
    *######,                .######*  ########        */////*.    .*/////*        ########.   *##################.
    *######,                ,######*  #######(         ,//////    //////,         (#######.   *##################,
    *######,                /######*  #######(           *////    ////*           (#######.   *##################,
    *######,            .,*(#######* .#######(            ,///    ///,            (#######.   *#######(((((((((((.
    *##############################, .########.            .//    //             .########.   *#######,
    *#############################(  .###########/*.         ,    ,         .*/###########.   *#######*
    *############################(    *###############*                  *###############*    *#######*
    *#########################(*.        .,/#############(/,        ,/(#############/,        *#######*
    *#####################/,.                 ./##############(  (##############/.            *#######*
                                                  .*/#########(  (#########/*.
                                                       ./#####(  (#####/.
                                                           .*((  ((*.
"""

import rospy
import socket
from datetime import datetime
from std_msgs.msg import String

class GetSockets(object):
    def __init__(self):
        self.socket_publisher = rospy.Publisher("open_sockets", String, queue_size=10)
        self.socket_mess = String()
        self.rate = rospy.Rate(5)

    def main_func(self):
        try:
            while not rospy.is_shutdown():
                listens = self.listen_sockets()

                if len(listens) > 2:
                    self.socket_mess.data = str(listens)
                    now = datetime.now()
                    #dt_string = now.strftime("%d/%m/%Y %H:%M:%S.%f")
                    dt_string = now.strftime("%H:%M:%S.%f")
                    print("\n\n[{0}] - Open Sockets = {1}\n\n".format(dt_string, self.socket_mess.data))
                    self.socket_publisher.publish(self.socket_mess)

                self.rate.sleep()

        except Exception as err:
            print("\n\nmain_func Error = {}\n\n".format(err))


    def listen_sockets(self, use_filter=True):
        listens = []
        lines = open("/proc/net/tcp").readlines()

        for l in lines:
            ls = l.split()

            if ls[3] == '0A':
                lp =  ls[1].split(':')
                ip = str(lp[0])
                pair = "%s.%s.%s.%s:%s" %( int(ip[6:8], 16), int(ip[4:6], 16), int(ip[2:4], 16), int(ip[0:2], 16), int(lp[1], 16))
                
                #if use_filter:
                #    if "0.0.0.0" not in pair:
                #        listens.append(str(pair))
                #else:
                #    listens.append(str(pair))
                listens.append(str(pair))

        return listens


if __name__ == '__main__':
    rospy.init_node('get_all_sockets_node', anonymous=True)
    task_class = GetSockets()
    task_class.main_func()
