from __future__ import annotations
import os
import asyncio
import signal
import subprocess
import xml.etree.ElementTree as ET
import time
import argparse
import logging, coloredlogs


logger = logging.getLogger(__name__)
coloredlogs.install(level=logging.DEBUG, logger=logger)


class RecordTrajectoryLauncher:
    def __init__(self) -> None:
        self.gazebo_process: subprocess.Popen = None
        self.mpc_process: subprocess.Popen = None
        self.record_process: subprocess.Popen = None

        self.devel_dir = '/home/jimmy/catkin_ws/devel'
        self.gazebo_launch_file = 'demo_world.launch'
        self.mpc_path = '/home/jimmy/catkin_ws/src/terra-simulation/terra_mpc/scripts'
        self.record_launch_file = 'record_data.launch'
        self.world_file = '/home/jimmy/catkin_ws/src/terra-simulation/terra_worlds/worlds/farm.world'
        self.env_config_path = '/home/jimmy/catkin_ws/src/terra-simulation/terra_worlds/configs'
        self.rosbag_path = '/home/jimmy/das_lab'
    
    def _sourced_command(self):
        '''
        Required source commands to run scripts within ROS environment
        '''
        return (
            f'source /opt/ros/noetic/setup.bash'
            + f' && source {self.devel_dir}/setup.bash'
        )
    
    def _change_world_sdf(self, env_idx: int):
        '''
        Overwrites existing world file with last include tag's URI updated to new environment's SDF.

        backup_world_file() should be called before and restore_world_file after this function has been
        called the desired number of times.

        Assumes the <include> for the plot sdf is last (as opposed to the heightmap <include>).

        Args:
            world_path (str): Path to the world file XML, e.g. ansel_experiment.world
            env_idx (int): Index of environments
        '''
        # Overwrites existing world file with
        tree = ET.parse(self.world_file)
        root = tree.getroot()

        include_elements = root.findall('.//include')
        if include_elements == []:
            raise RuntimeError('No include elements found')

        last_include_element = include_elements[-1]

        uri = last_include_element.find('uri')
        if uri is None:
            raise RuntimeError('No uri element found in last include element')

        # Change URI text
        logger.info(f'Updating world file [{self.world_file}] with environment [{env_idx}] ...')
        uri.text = f'model://env_{env_idx}'

        tree.write(self.world_file)  # Write the updated world file

    def _launch_env(self):
        '''
        Launch the gazebo simulator for the current environment config
        '''
        logger.info(f'Launching gazebo simulator ...')

        source_command = self._sourced_command()
        launch_command = source_command + f' && roslaunch terra_gazebo {self.gazebo_launch_file}'

        return subprocess.Popen(
            launch_command,
            shell=True, # Bash is necessary to run ROS
            executable='/bin/bash', 
            preexec_fn=os.setsid
        )
    
    def _launch_mpc(self):
        '''
        Launch the MPC node
        '''
        logger.info(f'Launching MPC node ...')

        source_command = self._sourced_command()
        launch_command = source_command + f' && cd {self.mpc_path}'
        launch_command += f' && python3 mpc_node.py'

        return subprocess.Popen(
            launch_command,
            shell=True, # Bash is necessary to run ROS
            executable='/bin/bash',
            preexec_fn=os.setsid
        )

    def _launch_record_data(self, env_idx: int):
        '''
        Launch the rowswitch node (record_data.launch)
        '''
        logger.info(f'Launching rowswitch node ...')

        source_command = self._sourced_command()
        env_config_path = self.env_config_path + f'/env_{env_idx}'
        rosbag_path = self.rosbag_path + f'/corn_env{env_idx}.bag'
        launch_command = source_command + f' && roslaunch terra_gazebo {self.record_launch_file} env_config_path:={env_config_path} rosbag_path:={rosbag_path}'

        return subprocess.Popen(
            launch_command,
            shell=True, # Bash is necessary to run ROS
            executable='/bin/bash',
            preexec_fn=os.setsid
        )

    def _stop_env(self):
        os.killpg(os.getpgid(self.gazebo_process.pid), signal.SIGTERM)  # Kill existing ROS node of gazebo
        logger.info('Waiting for gazebo simulator to shut down ...')
        time.sleep(3)
        os.system('pkill -f gz && pkill -f gazebo')  # Kill the simulator
        time.sleep(3)
        logger.info('Gazebo simulator is shutted down!')
        self.gazebo_process = None
    
    def _stop_mpc(self):
        os.killpg(os.getpgid(self.mpc_process.pid), signal.SIGTERM)  # Kill the MPC node
        logger.info('Waiting for MPC node to shut down ...')
        time.sleep(3)
        logger.info('MPC node is shutted down!')
        self.mpc_process = None
    
    def _stop_record_data(self):
        os.killpg(os.getpgid(self.record_process.pid), signal.SIGTERM)  # Kill the rowswitch node
        logger.info('Waiting for rowswitch node to shut down ...')
        time.sleep(3)
        logger.info('rowswitch node is shutted down!')
        self.record_process = None

    async def start_collection(self, env_start_idx: int, env_end_idx: int):
        for env_idx in range(env_start_idx, env_end_idx + 1, 1):
            self._change_world_sdf(env_idx=env_idx)  # Update environment world file
            logger.info(f'Launching environment ...')
            self.gazebo_process = self._launch_env() # Launch simulator
            time.sleep(15)
            logger.info(f'Gazebo simulator is launched!')
            self.mpc_process = self._launch_mpc()  # Launch MPC node
            time.sleep(5)
            logger.info(f'MPC node is launched!')
            self.record_process = self._launch_record_data(env_idx=env_idx)  # Launch rowswitch node
            logger.info(f'rowswitch node is launched!')

            time.sleep(300)  # Run simulation

            logger.info('Shutting down environment ...')
            self._stop_mpc() # Kill MPC node
            time.sleep(2)
            self._stop_record_data()  # Kill rowswitch node
            time.sleep(2)
            self._stop_env()  # Kill gazebo simulator
            time.sleep(2)
            logger.info('Successfully shutted down environment!')

            time.sleep(10)  # Switching to the next environment


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--env_start_idx', type=int, default=0, help='Index of environment to start collecting data')
    parser.add_argument('--env_end_idx', type=int, default=0, help='Index of environment when stop collecting data')
    args = parser.parse_args()

    launcher = RecordTrajectoryLauncher()

    asyncio.run(launcher.start_collection(args.env_start_idx, args.env_end_idx))
    logging.shutdown()