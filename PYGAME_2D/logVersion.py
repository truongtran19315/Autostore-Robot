import os
import datetime


def getlogVersion(dest_folder):
    current_datetime = datetime.datetime.now()
    folder_name = current_datetime.strftime("%Y-%m-%d")
    allFolder = os.listdir(dest_folder)
    print(f" Folder : {(allFolder)}")
    version_number = 1

    done_folders = [
        folder for folder in allFolder if folder_name in folder and "_DONE" in folder]

    if done_folders:
        version_numbers_list = [int(folder.split('_V')[-1][0])
                                for folder in done_folders]
        version_number = max(version_numbers_list) + 1

    folder_name += "_V" + str(version_number)
    return os.path.join(dest_folder, folder_name)
