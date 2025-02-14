import pygame

# 初始化 Pygame
pygame.init()

# 设置屏幕尺寸
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Square Move Up and Down")

# 定义正方形的初始位置和大小
square_size = 50
square_x = (screen_width - square_size) // 2
square_y = (screen_height - square_size) // 2
square_speed = 5

# 游戏主循环标志
running = True

while running:
    # 处理事件
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                # 按下上箭头键，正方形向上移动
                square_y -= square_speed
            elif event.key == pygame.K_DOWN:
                # 按下下箭头键，正方形向下移动
                square_y += square_speed

    # 边界检查，防止正方形移出屏幕
    if square_y < 0:
        square_y = 0
    elif square_y > screen_height - square_size:
        square_y = screen_height - square_size

    # 填充屏幕背景色
    screen.fill((255, 255, 255))

    # 绘制正方形
    pygame.draw.rect(screen, (255, 0, 0), (square_x, square_y, square_size, square_size))

    # 更新屏幕显示
    pygame.display.flip()

# 退出 Pygame
pygame.quit()