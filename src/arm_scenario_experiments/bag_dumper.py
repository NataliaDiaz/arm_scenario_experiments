import os
import shutil
import time

import rospy
import rosbag
import roslaunch

from sensor_msgs.msg import (JointState, Image, CompressedImage)

class BagDumper:
    button_pos = None

    def __init__(self, bag_path, dump_folder = None):
        self.bag_path = bag_path
        self.dump_folder = dump_folder or os.path.realpath(os.path.splitext(bag_path)[0])
        os.system('mkdir -p '+self.dump_folder)
        self.synchronized_topics = set()
        self.initialized = set()
        self.initializers = {}
        self.actions = {}
        self.lastMessage = {}
        self.margin_ok = 0.5 # if messages are found in all topics in an time interval smaller than this margin, these messages are saved to disk as the same timestep
        self.margin_warning = 0.7
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

    def __del__(self):
        self.launch.stop()


    def run(self, check = None):
        # first check if the bag is corrupted
        try:
            bag = rosbag.Bag(self.bag_path)
            bag.close()
        except rosbag.ROSBagUnindexedException:
            os.system('rosbag reindex '+self.bag_path)

        try:
            bag = rosbag.Bag(self.bag_path)
            self.lastMessage = {topic:None for topic in self.actions.keys()}
            nTopics = len(self.synchronized_topics)
            time = 0

            for topic, msg, t in bag.read_messages():
                if not topic in self.actions: continue
                if rospy.is_shutdown():break
                if topic not in self.initialized:
                    self.initializers[topic]()
                    self.initialized.add(topic)

                if self.lastMessage[topic]:
                    time = 'unknown'
                    if hasattr(msg, 'header'): time = self.lastMessage[topic].header.stamp.secs + self.lastMessage[topic].header.stamp.nsecs*1e-9
                    print('WARNING ! , a message around '+str(time)+' has been discarded')
                self.lastMessage[topic] = msg

                if topic in self.synchronized_topics:
                    mini = float("inf")
                    maxi = float("-inf")
                    counter = 0
                    for topic in self.synchronized_topics:
                        msg = self.lastMessage[topic]
                        if not msg: break
                        counter +=1
                        time = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
                        mini = min(mini, time)
                        maxi = max(maxi,time)
                    delay = maxi-mini

                    if counter==nTopics and 0<delay and delay<self.margin_warning:
                        if delay>self.margin_ok: print('Warning, messages around '+str(time)+' are more than '+str(self.margin_ok)+' apart')
                        if check: check(self.lastMessage)
                        self.dumpBuffers(self.synchronized_topics)
                else:
                    self.dumpBuffers([topic])
        finally:
            bag.close()

    def dumpBuffers(self, topics):
        topics = topics or self.lastMessage.keys()
        for topic in topics:
            msg = self.lastMessage[topic]
            self.actions[topic](msg)
            self.lastMessage[topic] = None


    def save_in_folder(self, topic, folder=None, synchronized = False):
        if synchronized: self.synchronized_topics.add(topic)
        folder = folder or os.path.join(self.dump_folder, topic.strip('/').replace('/','_') )
        time_file = os.path.join(folder,'time.txt')

        def initializer():
            if os.path.isdir(folder): shutil.rmtree(folder)
            os.system('mkdir -p '+folder)
            os.system('echo time > '+time_file)
        self.initializers[topic] = initializer

        words = topic.split('/')
        if words[-1]=='compressed':
            base_topic = '/'.join(words[:-1])
            compressed = ' compressed'
            message_class = CompressedImage
        else:
            base_topic = '/'.join(words)
            compressed = ''
            message_class = Image

        prefix = '/disk_writer_'+str(time.time()).replace('.','_')
        new_topic = (prefix+topic).replace('//','/')
        new_base_topic = (prefix+base_topic).replace('//','/')

        full_filename_format = os.path.join(folder, 'frame%05i.jpg')
        arg = 'image:='+new_base_topic+compressed+' _sec_per_frame:=0 _filename_format:='+full_filename_format
        rec = self.launch.launch(roslaunch.core.Node('image_view', 'extract_images', args=arg ))
        print("launching image_view extract_images "+arg)
        time.sleep(0.2) #to let the recorder some time to start
        publisher = rospy.Publisher(new_topic, message_class, latch=True, queue_size=1)
        print("\npublishing on new topic :: "+ new_base_topic+"\n")
        time.sleep(0.2)

        def action(msg):
            before = getNumberOfFilesIn(folder)
            publisher.publish(msg)
            waitForNewFile(folder, before)
            t = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
            os.system('echo {0:010.3f} >> {1}'.format(t, time_file) )
        self.actions[topic] = action


    def write_in_file(self, topic, headers, msg_handler, filename=None, synchronized = False):
        if synchronized: self.synchronized_topics.add(topic)
        filename = filename or os.path.join(self.dump_folder, topic.strip('/').replace('/','_')+'.txt' )

        def initializer():
            os.system("echo '# "+(' '.join(headers))+"' > "+filename)
        self.initializers[topic] = initializer

        def action(msg):
            os.system("echo "+msg_handler(msg)+" >> "+filename)
        self.actions[topic] = action

def getNumberOfFilesIn(path):
    return len(os.listdir(path))

def waitForNewFile(path, before):
    tstart = time.time()
    while(getNumberOfFilesIn(path)<=before) and not rospy.is_shutdown():
        rospy.sleep(0.001)
        if time.time()-tstart>2:
            print('Aborting')
            return False
    return True
