import os
import datetime
import cv2
from consts import GAME_SETTING

class Logger:
    def __init__(self, base_path):
        self.base_path = base_path
        self.video_folder_path = None
        self.image_folder_path = None
        self.log_folder_path = None

    def _get_log_version(self):
        current_datetime = datetime.datetime.now()
        folder_name = current_datetime.strftime("%Y-%m-%d")
        all_folders = os.listdir(self.base_path)
        version_number = 1

        done_folders = [folder for folder in all_folders if folder_name in folder and "_DONE" in folder]

        if done_folders:
            version_numbers_list = [int(folder.split('_V')[-1][0]) for folder in done_folders]
            version_number = max(version_numbers_list) + 1

        folder_name += "_V" + str(version_number)
        return os.path.join(self.base_path, folder_name)

    def _get_video_path(self, version_folder):
        all_files = os.listdir(version_folder)
        current_video_path = 'recordVideo'
        video_record_files = [file for file in all_files if "recordVideo" in file]

        if not video_record_files:
            current_video_path += '_0.avi'
        else:
            version_file_video_list = [int(file.split('_')[-1][0]) for file in video_record_files]
            version_number = max(version_file_video_list) + 1
            current_video_path += '_' + str(version_number) + '.avi'

        return os.path.join(version_folder, current_video_path)

    def _get_position_path(self, version_folder, done, version):
        if done == 0:
            current_video_path = 'TP_' + str(version) + '_.png'
        elif done == 1:
            current_video_path = 'TP_' + str(version) + '_DONE.png'
        elif done == 2:
            current_video_path = 'TP_' + str(version) + '_GOAL.png'
        return os.path.join(version_folder, current_video_path)

    def _get_log_path(self, version_folder, version):
        current_log_path = 'log_' + str(version) + '_.txt'
        return os.path.join(version_folder, current_log_path)

    def create_log_folders(self):
        folder_path = self._get_log_version()
        os.makedirs(folder_path, exist_ok=True)

        self.video_folder_path = os.path.join(folder_path, 'VIDEO')
        self.image_folder_path = os.path.join(folder_path, 'IMAGE')
        self.log_folder_path = os.path.join(folder_path, 'LOG')
        os.makedirs(self.video_folder_path, exist_ok=True)
        os.makedirs(self.image_folder_path, exist_ok=True)
        os.makedirs(self.log_folder_path, exist_ok=True)

        return folder_path

    def create_video_writer(self, version_folder):
        video_file_path = self._get_video_path(version_folder)
        return cv2.VideoWriter(video_file_path, cv2.VideoWriter_fourcc(*'MJPG'), GAME_SETTING.FPS,
                               (GAME_SETTING.SCREEN_WIDTH, GAME_SETTING.SCREEN_HEIGHT))

    def get_position_path(self, version_folder, done, version):
        return self._get_position_path(version_folder, done, version)

    def get_log_path(self, version_folder, version):
        return self._get_log_path(version_folder, version)
    
    # Phương thức getter
    def get_video_folder_path(self):
        return self.video_folder_path
    
    def get_image_folder_path(self):
        return self.image_folder_path
    
    def get_log_folder_path(self):
        return self.log_folder_path


# Sử dụng Logger
base_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'DATA')
logger = Logger(base_path)
log_folder = logger.create_log_folders()
video_writer = logger.create_video_writer(logger.get_video_folder_path())

# Lấy các đường dẫn thư mục
video_folder_path = logger.get_video_folder_path()
image_folder_path = logger.get_image_folder_path()
log_folder_path = logger.get_log_folder_path()