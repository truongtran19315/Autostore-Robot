screen = np.zeros((720, 1280, 3), dtype=np.uint8)
game = PyGame2D(screen)
while True:
    screen = np.zeros((720, 1280, 3), dtype=np.uint8)
    a = Utils.inputUser(game)
    game.view(screen)
    if not game.robot.isAlive:
        print("Oops!!!!!!!!!!")
        break
    elif game.robot.achieveGoal:
        print("Great!!!!!!!!!")
        break
    if a == False:
        cv2.imshow('min', game.saveMin["screen"])
        cv2.waitKey(0)
        break
    pass