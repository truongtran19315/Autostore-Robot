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


def getlogPosition_path(Version_folder, done):
    allfile = os.listdir(Version_folder)
    current_video_path = 'TP'  # tracking Position
    videoRecord_file = [file for file in allfile if "TP" in file]

    if not videoRecord_file:
        current_video_path += '_0_.png'
    else:
        version_fileVideo_list = [int(file.split('_')[1])
                                  for file in videoRecord_file]
        version_number = max(version_fileVideo_list) + 1
        if not done:
            current_video_path += '_' + str(version_number) + '_.png'
        else:
            current_video_path += '_' + str(version_number) + '_DONE.png'

    return os.path.join(Version_folder, current_video_path)
# temp_folder = 'C:\\Users\\truon\\PROJECTS\\PYTHON\\do-an-hk231\\Autostore-Robot\\PYGAME_2D\\DATA\\2023-11-15_V1'
# print(f"all file : {os.listdir(temp_folder)}")
# print(getlogVideo_path(temp_folder))
