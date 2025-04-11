import paramiko

def main(args=None):

    ip_addresses = ['192.168.0.102', '192.168.0.103', '192.168.0.104', '192.168.0.105', '192.168.0.106', '192.168.0.107', '192.168.0.108']
    user = 'rlab'
    pw = input("Give password: ")
    
    try:
    
        for i in ip_addresses:
        
            id = i[-1]
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(ip_address[i], username=user, password=pw)
            print(f"Connected, now running ros2 run command with id {id}")
        
            ssh.exec_command('cd ros2_ws')
            ssh.exec_command('source install/setup.bash')
            ssh.exec_command('colcon build')
            ssh.exec_command(f'ros2 run image_subscriber {id}')
            ssh.close()
        
    except Exception as e:
        print(f"An error occured with id {id}")
            
if __name__ == '__main__':
    main()
