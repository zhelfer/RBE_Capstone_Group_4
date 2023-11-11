import pygame

pygame.init()

white = (255, 255, 255)
forest = (80, 100, 54)
red = (255, 0, 0)
emerald = (0, 255, 0)
green = (67, 143, 2)
black = (0, 0, 0)
grey = (79, 79, 79)
deepblue = (24, 116, 205)
tree = (50, 0, 0)

min_x = 0
max_x = 500
min_y = 0
max_y = 500

displayscreen = pygame.display.set_mode((max_x, max_y))
pygame.display.set_caption('Lawnmower SAR')
clock = pygame.time.Clock()

x = 0
x2 = x
y = 0
y2 = y


def move():
    global y
    global x
    global x2
    global y2


def grid():
    block = 11
    for x in range(0, max_x, block):
        for y in range(0, max_y, block):
            rect = pygame.Rect(x, y, block, block)
            pygame.draw.rect(displayscreen, grey, rect, 1)


run = True
while run:

    clock.tick(15)
    displayscreen.fill(forest)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

    move()
    grid()

    image = pygame.image.load('Environment.png')
    DFLT_IMG_SZ = (500, 500)
    img = pygame.transform.scale(image, DFLT_IMG_SZ)
    imagerect = img.get_rect()
    displayscreen.blit(img,imagerect)

    pygame.draw.rect(displayscreen, deepblue, (x, y, 10, 10))

    if x % 2 == 0 and x < max_x:
        y = y + 11
        if (y) >= max_y:
            x = x2 + 11
            x2 = x

    elif x % 2 != 0 and x < max_x:
        y = y - 11
        if (y) == min_y - 11:
            x = x2 + 11
            x2 = x

    else:
        break

    pygame.display.update()

pygame.quit()
quit()