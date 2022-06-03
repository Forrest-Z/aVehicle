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

import pyttsx3
import threading

class SpeechClass(object):
    """
        10 - > id=default
        11 - > id=english
        13 - > id=english-north
        14 - > id=english_rp
        15 - > id=english_wmids
        16 - > id=english-us
        63 - > id=turkish
    """
    def __init__(self, speech_rate=175):
        self.engine = pyttsx3.init()
        self.engine.setProperty("rate", speech_rate)    #175        # Rate; 300 Speed, 100 Slow
        voices = self.engine.getProperty("voices")

        #self.engine.setProperty("voice", voices[63].id)       # Turkish
        self.engine.setProperty("voice", 'english_rp+f3')


    def get_languages(self):
        for index, voice in enumerate(self.engine.getProperty('voices')):
            print("{0} - > Voice Languages = {1}".format(index, voice.id))


    def __set_text_and_run_speech_func(self, text):
        try:
            if isinstance(text, list):
                for item in text:
                    self.engine.say(item)

            elif isinstance(text, str):
                self.engine.say(text)

            else:
                return

            # play the speech
            self.engine.runAndWait()

        except Exception as err:
            print("__set_text_and_run_speech_func Error = {}!".format(err))


    def set_text_and_run_speech_func(self, text):
        try:
            thread = threading.Thread(target=self.__set_text_and_run_speech_func, args=(text, ))
            thread.daemon = True
            thread.start()

        except Exception as err:
            print("set_text_and_run_speech_func Error = {}!".format(err))


if __name__ == '__main__':
    try:
        sc_class = SpeechClass()

        text_list = ["Hello World", "This is Spartaaaa"]
        text = "This is test text"
        text = "Uuuuu Uuuuu Uuuuu Erdoovaaannnn Erdoovaaannnn Erdoovaaannnn"
        sc_class.set_and_run_fun(text)

    except Exception as err:
        print("__main__ Error = {}!".format(err))
