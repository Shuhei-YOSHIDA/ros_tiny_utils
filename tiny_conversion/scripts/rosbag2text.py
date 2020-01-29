#!/usr/bin/env python3

import sys
import rospy
import rosbag

def _usage_exit():
    print("usage: rosbag2text.py path_to_rosbag [output_directory]")
    print("Default output_directory is /tmp")
    sys.exit(1)

def open_rosbag(filepath):
    try:
        bag = rosbag.Bag(filepath)
    except Exception as e:
        print('Error: filepath of rosbag seems to be wrong')
        _usage_exit()

    if bag == None:
        _usage_exit()

    return bag

def convert_to_textfiles(bag, directory_path):
    topic_info = bag.get_type_and_topic_info()
    all_topic_names = list(topic_info.topics.keys())
    print('all_topic_names', all_topic_names)
    all_topic_types = [topic_info.topics[n].msg_type for n in all_topic_names]
    print('all_topic_types', all_topic_types)
    all_topic_md5 = [topic_info.msg_types[t] for t in all_topic_types]
    print('all_topic_md5', all_topic_md5)

    # directory_path = '/tmp'
    file_list = [open(directory_path + '/' + name + ".txt", mode='w') for name in all_topic_names]
    for idx, (name, msg_type, md5) in enumerate(zip(all_topic_names, all_topic_types, all_topic_md5)):
        file_list[idx].write('# topicname ' + name + "\n")
        file_list[idx].write('# topictype ' + msg_type + "\n")
        file_list[idx].write('# topicmd5 ' + md5 + "\n")
        file_list[idx].write('# messagenum ' + str(bag.get_message_count(topic_filters=[name])) + "\n")

        for msg_num, (topic, msg, t) in enumerate(bag.read_messages(topics=[name])):
            file_list[idx].write('### message : ' + str(msg_num) + " : time [" + str(t) + "]" + "\n")
            file_list[idx].write(str(msg) + "\n")

        file_list[idx].write('# message '+ str(bag.get_message_count(topic_filters=[name])) + "\n")

        file_list[idx].close()

def main():
    if (len(sys.argv) < 2):
        _usage_exit()
    bag = open_rosbag(sys.argv[1])
    print("loaded file : " + sys.argv[1] )

    try:
        directory_path = sys.argv[2]
    except Exception as e:
        directory_path = "/tmp"
    print("output_directory path is set to " + directory_path)

    try:
        convert_to_textfiles(bag, directory_path)
    except Exception as e:
        print(e)
        print("failed conversion")
        _usage_exit()

if __name__ == "__main__":
    main()
