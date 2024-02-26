import os
import datetime


def getlogVersion(dest_folder):
    current_datetime = datetime.datetime.now()
    folder_name = current_datetime.strftime("%Y-%m-%d")
    allFolder = os.listdir(dest_folder)
    # print(f" Folder : {(allFolder)}")
    version_number = 1

    done_folders = [
        folder for folder in allFolder if folder_name in folder and "_DONE" in folder]

    if done_folders:
        version_numbers_list = [int(folder.split('_V')[-1][0])
                                for folder in done_folders]
        version_number = max(version_numbers_list) + 1

    folder_name += "_V" + str(version_number)
    return os.path.join(dest_folder, folder_name)


def getlogVideo_path(Version_folder):
    allfile = os.listdir(Version_folder)
    current_video_path = 'recordVideo'
    videoRecord_file = [file for file in allfile if "recordVideo" in file]

    if not videoRecord_file:
        current_video_path += '_0.avi'
    else:
        version_fileVideo_list = [int(file.split('_')[-1][0])
                                  for file in videoRecord_file]
        version_number = max(version_fileVideo_list) + 1
        current_video_path += '_' + str(version_number) + '.avi'

    return os.path.join(Version_folder, current_video_path)


def getlogPosition_path(Version_folder, done, version):
    if done == 0:
        current_video_path = 'TP_' + str(version) + '_.png'
    elif done == 1:
        current_video_path = 'TP_' + str(version) + '_DONE.png'
    elif done == 2:
        current_video_path = 'TP_' + str(version) + '_GOAL.png'
    return os.path.join(Version_folder, current_video_path)


def getLog_path(log_folder, version):
    current_log_path = 'log_' + str(version) + '_.txt'
    return os.path.join(log_folder, current_log_path)


base_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'DATA')
os.makedirs(base_path, exist_ok=True)


folder_path = getlogVersion(base_path)
os.makedirs(folder_path, exist_ok=True)

videoFolderPath = os.path.join(folder_path, 'VIDEO')
imageFolderPath = os.path.join(folder_path, 'IMAGE')
logFolderPath = os.path.join(folder_path, 'LOG')
os.makedirs(videoFolderPath, exist_ok=True)
os.makedirs(imageFolderPath, exist_ok=True)
os.makedirs(logFolderPath, exist_ok=True)

# log_path = getLog_path(logFolderPath)
# os.makedirs(log_path)
# with open(log_path, 'a') as file:
#     print(f'//////////////////////////// Log file /////////////////////////////', file=file)

# os.chmod(log_path, stat.S_IWRITE)
