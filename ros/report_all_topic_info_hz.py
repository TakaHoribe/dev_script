import subprocess
import time
import select

def get_topic_list():
    print("Getting list of topics...")
    result = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE)
    topics = result.stdout.decode('utf-8').strip().split('\n')
    print(f"Found {len(topics)} topics.")
    return topics

def get_hz(topic):
    print(f"Checking Hz for topic: {topic}...")
    command = f'ros2 topic hz -w 10 {topic}'
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True, executable='/bin/bash', text=True)
    
    start_time = time.time()
    hz = 0.0

    timeout_sec = 5

    while True:
        ready = select.select([process.stdout], [], [], 0.1)[0]
        if ready:
            output = process.stdout.readline()
            # print(f"Output: {output.strip()}")  # デバッグ用print
            if "average rate" in output:
                try:
                    hz = float(output.split()[-1])
                    print(f"Topic: {topic}, Hz: {hz:.2f}")
                    break
                except ValueError as e:
                    print(f"ValueError: {e}")
                    break
        elif time.time() - start_time > timeout_sec:
            print(f"Topic: {topic} timed out, assuming Hz: 0.00")
            break
        else:
            time.sleep(0.1)

    process.terminate()
    return hz

def get_info(topic):
    print(f"Checking info for topic: {topic}...")
    command = f'ros2 topic info {topic}'
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True, executable='/bin/bash', text=True)
    
    start_time = time.time()
    timeout_sec = 5

    pub_ok = False
    sub_ok = False
    pub_num = 0
    sub_num = 0

    while True:
        ready = select.select([process.stdout], [], [], 0.1)[0]
        if ready:
            output = process.stdout.readline()
            # print(f"Output: {output.strip()}")  # デバッグ用print
            if "Publisher" in output:
                try:
                    pub_num = float(output.split()[-1])
                    print(f"Topic: {topic}, Publisher Count: {pub_num:.0f}")
                    pub_ok = True
                except ValueError as e:
                    print(f"ValueError: {e}")
            if "Subscription" in output:
                try:
                    sub_num = float(output.split()[-1])
                    print(f"Topic: {topic}, Subscription Count: {sub_num:.0f}")
                    sub_ok = True
                except ValueError as e:
                    print(f"ValueError: {e}")
                    break
            if pub_ok and sub_ok:
                break
        elif time.time() - start_time > timeout_sec:
            print(f"Topic: {topic} timed out, assuming Pub = 0, Sub = 0\n")
            break
        else:
            time.sleep(0.1)

    process.terminate()
    return pub_num, sub_num

def main():
    topics = get_topic_list()
    topic_hz_list = []

    for i, topic in enumerate(topics):
        hz = get_hz(topic)
        pub_num, sub_num = get_info(topic)
        topic_hz_list.append((topic, hz, pub_num, sub_num))
        print(f"Progress: {i+1}/{len(topics)} topics processed.")
        print("\n")

    print("Sorting topics by Hz...")
    topic_hz_list.sort(key=lambda x: x[1], reverse=True)

    print("Printing sorted topics:")
    for topic, hz, pub_num, sub_num in topic_hz_list:
        print(f'{topic}, {hz:.2f}, {pub_num:.0f}, {sub_num:.0f}')

if __name__ == '__main__':
    main()