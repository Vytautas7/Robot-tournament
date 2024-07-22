from cmath import inf, sqrt
import numpy as np

def strategy(blocks, we, them, goal_center_us, enemy_has_block, enemy_get_block):
    xg1 = [[inf,inf],
            [inf,inf]]
    yg1 = [[inf,inf],
            [inf,inf]]
    xo1 = [[inf,inf],
            [inf,inf]]
    yo1 = [[inf,inf],
            [inf,inf]]
    xb1 = [[inf,inf],
            [inf,inf]]
    yb1 = [[inf,inf],
            [inf,inf]]
    g_u_g2 = [[None, None],
            [None, None]]
    g_t_g2 = [[None, None],
            [None, None]]
    g_u_o2 = [[None, None],
            [None, None]]
    g_t_o2 = [[None, None],
            [None, None]]
    g_u_b2 = [[None, None],
            [None, None]]
    g_t_b2 = [[None, None],
            [None, None]]
    for i in range(len(blocks)):
        keys = list(blocks.keys())
        if blocks[keys[i]][2] == 1:
            x_g = np.array([blocks[keys[i]][0]])
            y_g = np.array([blocks[keys[i]][1]])
            g_u_g1 = np.array([blocks[keys[i]][3]])
            g_t_g1 = np.array([blocks[keys[i]][4]])
            xg1.append(x_g)
            yg1.append(y_g)
            g_u_g2.append(g_u_g1)
            g_t_g2.append(g_t_g1)
        if blocks[keys[i]][2] == 2:
            x_o = np.array([blocks[keys[i]][0]])
            y_o = np.array([blocks[keys[i]][1]])
            g_u_o1 = np.array([blocks[keys[i]][3]])
            g_t_o1 = np.array([blocks[keys[i]][4]])
            xo1.append(x_o)
            yo1.append(y_o)
            g_u_o2.append(g_u_o1)
            g_t_o2.append(g_t_o1)
        if blocks[keys[i]][2] == 3:
            x_b = np.array([blocks[keys[i]][0]])
            y_b = np.array([blocks[keys[i]][1]])
            g_u_b1 = np.array([blocks[keys[i]][3]])
            g_t_b1 = np.array([blocks[keys[i]][4]])
            xb1.append(x_b)
            yb1.append(y_b)
            g_u_b2.append(g_u_b1)
            g_t_b2.append(g_t_b1)
    Wg = 1      #Weight of the Green block
    Wo = 2      #Weight of the Orange block
    Wb = 3      #Weight of the Blue block
    ###Define current position (x,y) (0,0) to (600,400)
    #Our robot (x,y,theta)
    x = np.array([we[0]])   #<------------------------------------------
    y = np.array([we[1]])   #<------------------------------------------
    #Enemy robot (x,y) 
    x_e = np.array([them[0]])   #<-------------------------------------------
    y_e = np.array([them[1]])   #<-------------------------------------------
    ###Define goal area position (assumed a center coordinate of the goal area)
    x_goal_us = np.array([goal_center_us[0]])  #
    y_goal_us = np.array([goal_center_us[1]])  # - position where the enemy is going to place the blocks
    ###Define the position and status of the blocks (x, y)
    infinite = np.array([
                        [inf,inf],
                        [inf,inf],
                        [inf,inf],
                        [inf,inf],
                        [inf,inf],
                        [inf,inf],
                        [inf,inf]
                            ])
    block_none = np.array([
                        [None, None],
                        [None, None],
                        [None, None],
                        [None, None],
                        [None, None],
                        [None, None],
                        [None, None]
                                    ])

    #Green blocks
    xg2 = np.concatenate(xg1)
    yg2 = np.concatenate(yg1)
    green_T = (xg2, yg2)
    greens = np.transpose(green_T)
    block_pos_g = np.concatenate((greens, infinite), axis = 0)
    x_n_g = block_pos_g[:,[0]]
    y_n_g = block_pos_g[:,[1]]
    ###Sorts the green blocks [goal_us, goal_them]
    g_u_g3 = np.concatenate(g_u_g2)
    g_t_g3 = np.concatenate(g_t_g2)
    goal_g_T = (g_u_g3, g_t_g3)
    goal_g = np.transpose(goal_g_T)
    #goal_greeny = np.concatenate((goal_g), axis = 0)
    goal_greens = np.concatenate((goal_g, block_none), axis = 0)
    #Orange blocks
    xo2 = np.concatenate(xo1)
    yo2 = np.concatenate(yo1)
    orange_T = (xo2, yo2)
    oranges = np.transpose(orange_T)
    block_pos_o = np.concatenate((oranges, infinite), axis = 0)
    x_n_o = block_pos_o[:,[0]]
    y_n_o = block_pos_o[:,[1]]
    ###Sorts the orange blocks [goal_us, goal_them]
    g_u_o3 = np.concatenate(g_u_o2)
    g_t_o3 = np.concatenate(g_t_o2)
    goal_o_T = (g_u_o3, g_t_o3)
    goal_o = np.transpose(goal_o_T)
    #goal_orangy = np.concatenate((goal_o), axis = 0)
    goal_oranges = np.concatenate((goal_o, block_none), axis = 0)
    #Blue blocks
    xb2 = np.concatenate(xb1)
    yb2 = np.concatenate(yb1)
    blue_T = (xb2, yb2)
    blues = np.transpose(blue_T)
    block_pos_b = np.concatenate((blues, infinite), axis = 0)
    x_n_b = block_pos_b[:,[0]]
    y_n_b = block_pos_b[:,[1]]
    ###Sorts the blue blocks [goal_us, goal_them]
    g_u_b3 = np.concatenate(g_u_b2)
    g_t_b3 = np.concatenate(g_t_b2)
    goal_b_T = (g_u_b3, g_t_b3)
    goal_b = np.transpose(goal_b_T)
    #goal_bluey = np.concatenate((goal_b), axis = 0)
    goal_blues = np.concatenate((goal_b, block_none), axis = 0)
    ###Find the distance from current position to the respective blocks
    #Our robot
    distance_to_block_g = np.sqrt(((x_n_g-x)**2)+(y_n_g-y)**2)
    distance_to_block_o = np.sqrt(((x_n_o-x)**2)+(y_n_o-y)**2)
    distance_to_block_b = np.sqrt(((x_n_b-x)**2)+(y_n_b-y)**2)
    #Enemy robot
    enemy_distance_to_block_g = np.sqrt(((x_n_g-x_e)**2)+(y_n_g-y_e)**2)
    enemy_distance_to_block_o = np.sqrt(((x_n_o-x_e)**2)+(y_n_o-y_e)**2)
    enemy_distance_to_block_b = np.sqrt(((x_n_b-x_e)**2)+(y_n_b-y_e)**2)
    ###Find the distance from our goal area to the respective blocks
    goal_us_distance_to_block_g = np.sqrt(((x_n_g-x_goal_us)**2)+(y_n_g-y_goal_us)**2)
    goal_us_distance_to_block_o = np.sqrt(((x_n_o-x_goal_us)**2)+(y_n_o-y_goal_us)**2)
    goal_us_distance_to_block_b = np.sqrt(((x_n_b-x_goal_us)**2)+(y_n_b-y_goal_us)**2)
    ###Weight the obtained distance with respect to the points of the block
    #Our robot
    weight_g = Wg / distance_to_block_g
    weight_o = Wo / distance_to_block_o
    weight_b = Wb / distance_to_block_b
    weight = (weight_g, weight_o, weight_b)
    total_weight = np.asarray(weight)
    weighted = np.concatenate((weight_g, weight_o, weight_b), axis=0)
    #Enemy robot
    enemy_weight_g = Wg / enemy_distance_to_block_g
    enemy_weight_o = Wo / enemy_distance_to_block_o
    enemy_weight_b = Wb / enemy_distance_to_block_b
    enemy_weight = (enemy_weight_g, enemy_weight_o, enemy_weight_b)
    enemy_total_weight = np.asarray(enemy_weight)
    enemy_weighted = np.concatenate((enemy_weight_g, enemy_weight_o, enemy_weight_b), axis=0)
    #Weighting the blocks from our goal area (where enemy places its blocks)
    goal_us_weight_g = Wg / goal_us_distance_to_block_g
    goal_us_weight_o = Wo / goal_us_distance_to_block_o
    goal_us_weight_b = Wb / goal_us_distance_to_block_b
    goal_us_weight = (goal_us_weight_g, goal_us_weight_o, goal_us_weight_b)
    goal_us_total_weight = np.asarray(goal_us_weight)
    goal_us_weighted = np.concatenate((goal_us_weight_g, goal_us_weight_o, goal_us_weight_b), axis=0)
    ###List the weights in descending order
    points = np.asarray(weighted)
    order = np.sort(points, axis=0)[::-1]
    enemy_points = np.asarray(enemy_weighted)
    enemy_order = np.sort(enemy_points, axis=0)[::-1]
    goal_us_points = np.asarray(goal_us_weighted)
    goal_us_order = np.sort(goal_us_points, axis=0)[::-1]
    #Obtain the row and column of the selected element
    selected1 = np.array(np.where(total_weight == order[0])).T
    selected2 = np.array(np.where(total_weight == order[1])).T
    selected3 = np.array(np.where(total_weight == order[2])).T
    selected4 = np.array(np.where(total_weight == order[3])).T
    selected5 = np.array(np.where(total_weight == order[4])).T
    selected6 = np.array(np.where(total_weight == order[5])).T
    selected7 = np.array(np.where(total_weight == order[6])).T
    enemy_selected1 = np.array(np.where(enemy_total_weight == enemy_order[0])).T
    enemy_selected2 = np.array(np.where(enemy_total_weight == enemy_order[1])).T
    enemy_selected3 = np.array(np.where(enemy_total_weight == enemy_order[2])).T
    enemy_selected4 = np.array(np.where(enemy_total_weight == enemy_order[3])).T
    enemy_selected5 = np.array(np.where(enemy_total_weight == enemy_order[4])).T
    enemy_selected6 = np.array(np.where(enemy_total_weight == enemy_order[5])).T
    enemy_selected7 = np.array(np.where(enemy_total_weight == enemy_order[6])).T
    goal_us_selected1 = np.array(np.where(goal_us_total_weight == goal_us_order[0])).T
    goal_us_selected2 = np.array(np.where(goal_us_total_weight == goal_us_order[1])).T
    goal_us_selected3 = np.array(np.where(goal_us_total_weight == goal_us_order[2])).T
    goal_us_selected4 = np.array(np.where(goal_us_total_weight == goal_us_order[3])).T
    goal_us_selected5 = np.array(np.where(goal_us_total_weight == goal_us_order[4])).T
    goal_us_selected6 = np.array(np.where(goal_us_total_weight == goal_us_order[5])).T
    goal_us_selected7 = np.array(np.where(goal_us_total_weight == goal_us_order[6])).T
    #Find the color of the block that it belongs to
    n1 = selected1[0,0]
    n2 = selected2[0,0]
    n3 = selected3[0,0]
    n4 = selected4[0,0]
    n5 = selected5[0,0]
    n6 = selected6[0,0]
    n7 = selected7[0,0]
    enemy_n1 = enemy_selected1[0,0]
    enemy_n2 = enemy_selected2[0,0]
    enemy_n3 = enemy_selected3[0,0]
    enemy_n4 = enemy_selected4[0,0]
    enemy_n5 = enemy_selected5[0,0]
    enemy_n6 = enemy_selected6[0,0]
    enemy_n7 = enemy_selected7[0,0]
    goal_us_n1 = goal_us_selected1[0,0]
    goal_us_n2 = goal_us_selected2[0,0]
    goal_us_n3 = goal_us_selected3[0,0]
    goal_us_n4 = goal_us_selected4[0,0]
    goal_us_n5 = goal_us_selected5[0,0]
    goal_us_n6 = goal_us_selected6[0,0]
    goal_us_n7 = goal_us_selected7[0,0]
    #Get a position row number of the selected element
    r1 = selected1[0,1]
    r2 = selected2[0,1]
    r3 = selected3[0,1]
    r4 = selected4[0,1]
    r5 = selected5[0,1]
    r6 = selected6[0,1]
    r7 = selected7[0,1]
    enemy_r1 = enemy_selected1[0,1]
    enemy_r2 = enemy_selected2[0,1]
    enemy_r3 = enemy_selected3[0,1]
    enemy_r4 = enemy_selected4[0,1]
    enemy_r5 = enemy_selected5[0,1]
    enemy_r6 = enemy_selected6[0,1]
    enemy_r7 = enemy_selected7[0,1]
    goal_us_r1 = goal_us_selected1[0,1]
    goal_us_r2 = goal_us_selected2[0,1]
    goal_us_r3 = goal_us_selected3[0,1]
    goal_us_r4 = goal_us_selected4[0,1]
    goal_us_r5 = goal_us_selected5[0,1]
    goal_us_r6 = goal_us_selected6[0,1]
    goal_us_r7 = goal_us_selected7[0,1]
    ###The first weighted selection:
    #Our robot
    if n1 == 0:
        selected_distance1 = distance_to_block_g[r1,0]
        selected_block_pos1 = block_pos_g[r1,:]
        selected_block_status1 = goal_greens[r1,1]
        color1 = "Green block, 1 point, "
    if n1 == 1:
        selected_distance1 = distance_to_block_o[r1,0]
        selected_block_pos1 = block_pos_o[r1,:]
        selected_block_status1 = goal_oranges[r1,1]
        color1 = "Orange block, 2 points, "
    if n1 == 2:
        selected_distance1 = distance_to_block_b[r1,0]
        selected_block_pos1 = block_pos_b[r1,:]
        selected_block_status1 = goal_blues[r1,1]
        color1 = "Blue block, 3 points, "
    #Enemy robot
    if enemy_n1 == 0:
        enemy_selected_distance1 = enemy_distance_to_block_g[enemy_r1,0]
        enemy_selected_block_pos1 = block_pos_g[enemy_r1,:]
        enemy_selected_block_status1 = goal_greens[enemy_r1,0]
        enemy_color1 = "Green block, 1 point, "
    if enemy_n1 == 1:
        enemy_selected_distance1 = enemy_distance_to_block_o[enemy_r1,0]
        enemy_selected_block_pos1 = block_pos_o[enemy_r1,:]
        enemy_selected_block_status1 = goal_oranges[enemy_r1,0]
        enemy_color1 = "Orange block, 2 points, "
    if enemy_n1 == 2:
        enemy_selected_distance1 = enemy_distance_to_block_b[enemy_r1,0]
        enemy_selected_block_pos1 = block_pos_b[enemy_r1,:]
        enemy_selected_block_status1 = goal_oranges[enemy_r1,0]
        enemy_color1 = "Blue block, 3 points, "
    #Enemy selecting the blocks from our goal area (where enemy places its blocks)
    if goal_us_n1 == 0:
        goal_us_selected_distance1 = goal_us_distance_to_block_g[goal_us_r1,0]
        goal_us_selected_block_pos1 = block_pos_g[goal_us_r1,:]
        goal_us_selected_block_status1 = goal_greens[goal_us_r1,0]
        goal_us_color1 = "Green block, 1 point, "
    if goal_us_n1 == 1:
        goal_us_selected_distance1 = goal_us_distance_to_block_o[goal_us_r1,0]
        goal_us_selected_block_pos1 = block_pos_o[goal_us_r1,:]
        goal_us_selected_block_status1 = goal_oranges[goal_us_r1,0]
        goal_us_color1 = "Orange block, 2 points, "
    if goal_us_n1 == 2:
        goal_us_selected_distance1 = goal_us_distance_to_block_b[goal_us_r1,0]
        goal_us_selected_block_pos1 = block_pos_b[goal_us_r1,:]
        goal_us_selected_block_status1 = goal_blues[goal_us_r1,0]
        goal_us_color1 = "Blue block, 3 points, "
    ###The second weighted selection:
    #Our robot
    if n2 == 0:
        selected_distance2 = distance_to_block_g[r2,0]
        selected_block_pos2 = block_pos_g[r2,:]
        selected_block_status2 = goal_greens[r2,1]
        color2 = "Green block, 1 point, "
    if n2 == 1:
        selected_distance2 = distance_to_block_o[r2,0]
        selected_block_pos2 = block_pos_o[r2,:]
        selected_block_status2 = goal_oranges[r2,1]
        color2 = "Orange block, 2 points, "
    if n2 == 2:
        selected_distance2 = distance_to_block_b[r2,0]
        selected_block_pos2 = block_pos_b[r2,:]
        selected_block_status2 = goal_blues[r2,1]
        color2 = "Blue block, 3 points, "
    #Enemy robot
    if enemy_n2 == 0:
        enemy_selected_distance2 = enemy_distance_to_block_g[enemy_r2,0]
        enemy_selected_block_pos2 = block_pos_g[enemy_r2,:]
        enemy_selected_block_status2 = goal_greens[enemy_r2,0]
        enemy_color2 = "Green block, 1 point, "
    if enemy_n2 == 1:
        enemy_selected_distance2 = enemy_distance_to_block_o[enemy_r2,0]
        enemy_selected_block_pos2 = block_pos_o[enemy_r2,:]
        enemy_selected_block_status2 = goal_oranges[enemy_r2,0]
        enemy_color2 = "Orange block, 2 points, "
    if enemy_n2 == 2:
        enemy_selected_distance2 = enemy_distance_to_block_b[enemy_r2,0]
        enemy_selected_block_pos2 = block_pos_b[enemy_r2,:]
        enemy_selected_block_status2 = goal_blues[enemy_r2,0]
        enemy_color2 = "Blue block, 3 points, "
    #Enemy selecting the blocks from our goal area (where enemy places its blocks)
    if goal_us_n2 == 0:
        goal_us_selected_distance2= goal_us_distance_to_block_g[goal_us_r2,0]
        goal_us_selected_block_pos2 = block_pos_g[goal_us_r2,:]
        goal_us_selected_block_status2 = goal_greens[goal_us_r2,0]
        goal_us_color2 = "Green block, 1 point, "
    if goal_us_n2 == 1:
        goal_us_selected_distance2 = goal_us_distance_to_block_o[goal_us_r2,0]
        goal_us_selected_block_pos2 = block_pos_o[goal_us_r2,:]
        goal_us_selected_block_status2 = goal_oranges[goal_us_r2,0]
        goal_us_color2 = "Orange block, 2 points, "
    if goal_us_n2 == 2:
        goal_us_selected_distance2 = goal_us_distance_to_block_b[goal_us_r2,0]
        goal_us_selected_block_pos2 = block_pos_b[goal_us_r2,:]
        goal_us_selected_block_status2 = goal_blues[goal_us_r2,0]
        goal_us_color2 = "Blue block, 3 points, "
    ###The third weighted selection:
    #Our robot
    if n3 == 0:
        selected_distance3 = distance_to_block_g[r3,0]
        selected_block_pos3 = block_pos_g[r3,:]
        selected_block_status3 = goal_greens[r3,1]
        color3 = "Green block, 1 point, "
    if n3 == 1:
        selected_distance3 = distance_to_block_o[r3,0]
        selected_block_pos3 = block_pos_o[r3,:]
        selected_block_status3 = goal_oranges[r3,1]
        color3 = "Orange block, 2 points, "
    if n3 == 2:
        selected_distance3 = distance_to_block_b[r3,0]
        selected_block_pos3 = block_pos_b[r3,:]
        selected_block_status3 = goal_blues[r3,1]
        color3 = "Blue block, 3 points, "
    #Enemy robot
    if enemy_n3 == 0:
        enemy_selected_distance3 = enemy_distance_to_block_g[enemy_r3,0]
        enemy_selected_block_pos3 = block_pos_g[enemy_r3,:]
        enemy_selected_block_status3 = goal_greens[enemy_r3,0]
        enemy_color3 = "Green block, 1 point, "
    if enemy_n3 == 1:
        enemy_selected_distance3 = enemy_distance_to_block_o[enemy_r3,0]
        enemy_selected_block_pos3 = block_pos_o[enemy_r3,:]
        enemy_selected_block_status3 = goal_oranges[enemy_r3,0]
        enemy_color3 = "Orange block, 2 points, "
    if enemy_n3 == 2:
        enemy_selected_distance3 = enemy_distance_to_block_b[enemy_r3,0]
        enemy_selected_block_pos3 = block_pos_b[enemy_r3,:]
        enemy_selected_block_status3 = goal_blues[enemy_r3,0]
        enemy_color3 = "Blue block, 3 points, "
    #Enemy selecting the blocks from our goal area (where enemy places its blocks)
    if goal_us_n3 == 0:
        goal_us_selected_distance3 = goal_us_distance_to_block_g[goal_us_r3,0]
        goal_us_selected_block_pos3 = block_pos_g[goal_us_r3,:]
        goal_us_selected_block_status3 = goal_greens[goal_us_r3,0]
        goal_us_color3 = "Green block, 1 point, "
    if goal_us_n3 == 1:
        goal_us_selected_distance3 = goal_us_distance_to_block_o[goal_us_r3,0]
        goal_us_selected_block_pos3 = block_pos_o[goal_us_r3,:]
        goal_us_selected_block_status3 = goal_oranges[goal_us_r3,0]
        goal_us_color3 = "Orange block, 2 points, "
    if goal_us_n3 == 2:
        goal_us_selected_distance3 = goal_us_distance_to_block_b[goal_us_r3,0]
        goal_us_selected_block_pos3 = block_pos_b[goal_us_r3,:]
        goal_us_selected_block_status3 = goal_blues[goal_us_r3,0]
        goal_us_color3 = "Blue block, 3 points, "

    ###The fourth weighted selection:
    #Our robot
    if n4 == 0:
        selected_distance4 = distance_to_block_g[r4,0]
        selected_block_pos4 = block_pos_g[r4,:]
        selected_block_status4 = goal_greens[r4,1]
        color4 = "Green block, 1 point, "
    if n4 == 1:
        selected_distance4 = distance_to_block_o[r4,0]
        selected_block_pos4 = block_pos_o[r4,:]
        selected_block_status4 = goal_oranges[r4,1]
        color4 = "Orange block, 2 points, "
    if n4 == 2:
        selected_distance4 = distance_to_block_b[r4,0]
        selected_block_pos4 = block_pos_b[r4,:]
        selected_block_status4 = goal_blues[r4,1]
        color4 = "Blue block, 3 points, "
    #Enemy robot
    if enemy_n4 == 0:
        enemy_selected_distance4 = enemy_distance_to_block_g[enemy_r4,0]
        enemy_selected_block_pos4 = block_pos_g[enemy_r4,:]
        enemy_selected_block_status4 = goal_greens[enemy_r4,0]
        enemy_color4 = "Green block, 1 point, "
    if enemy_n4 == 1:
        enemy_selected_distance4 = enemy_distance_to_block_o[enemy_r4,0]
        enemy_selected_block_pos4 = block_pos_o[enemy_r4,:]
        enemy_selected_block_status4 = goal_oranges[enemy_r4,0]
        enemy_color4 = "Orange block, 2 points, "
    if enemy_n4 == 2:
        enemy_selected_distance4 = enemy_distance_to_block_b[enemy_r4,0]
        enemy_selected_block_pos4 = block_pos_b[enemy_r4,:]
        enemy_selected_block_status4 = goal_blues[enemy_r4,0]
        enemy_color4 = "Blue block, 3 points, "
    #Enemy selecting the blocks from our goal area (where enemy places its blocks)
    if goal_us_n4 == 0:
        goal_us_selected_distance4 = goal_us_distance_to_block_g[goal_us_r4,0]
        goal_us_selected_block_pos4 = block_pos_g[goal_us_r4,:]
        goal_us_selected_block_status4 = goal_greens[goal_us_r4,0]
        goal_us_color4 = "Green block, 1 point, "
    if goal_us_n4 == 1:
        goal_us_selected_distance4 = goal_us_distance_to_block_o[goal_us_r4,0]
        goal_us_selected_block_pos4 = block_pos_o[goal_us_r4,:]
        goal_us_selected_block_status4 = goal_oranges[goal_us_r4,0]
        goal_us_color4 = "Orange block, 2 points, "
    if goal_us_n4 == 2:
        goal_us_selected_distance4 = goal_us_distance_to_block_b[goal_us_r4,0]
        goal_us_selected_block_pos4 = block_pos_b[goal_us_r4,:]
        goal_us_selected_block_status4 = goal_blues[goal_us_r4,0]
        goal_us_color4 = "Blue block, 3 points, "

    ###The fifth weighted selection:
    #Our robot
    if n5 == 0:
        selected_distance5 = distance_to_block_g[r5,0]
        selected_block_pos5 = block_pos_g[r5,:]
        selected_block_status5 = goal_greens[r5,1]
        color5 = "Green block, 1 point, "
    if n5 == 1:
        selected_distance5 = distance_to_block_o[r5,0]
        selected_block_pos5 = block_pos_o[r5,:]
        selected_block_status5 = goal_oranges[r5,1]
        color5 = "Orange block, 2 points, "
    if n5 == 2:
        selected_distance5 = distance_to_block_b[r5,0]
        selected_block_pos5 = block_pos_b[r5,:]
        selected_block_status5 = goal_blues[r5,1]
        color5 = "Blue block, 3 points, "
    #Enemy robot
    if enemy_n5 == 0:
        enemy_selected_distance5 = enemy_distance_to_block_g[enemy_r5,0]
        enemy_selected_block_pos5 = block_pos_g[enemy_r5,:]
        enemy_selected_block_status5 = goal_greens[enemy_r5,0]
        enemy_color5 = "Green block, 1 point, "
    if enemy_n5 == 1:
        enemy_selected_distance5 = enemy_distance_to_block_o[enemy_r5,0]
        enemy_selected_block_pos5 = block_pos_o[enemy_r5,:]
        enemy_selected_block_status5 = goal_oranges[enemy_r5,0]
        enemy_color5 = "Orange block, 2 points, "
    if enemy_n5 == 2:
        enemy_selected_distance5 = enemy_distance_to_block_b[enemy_r5,0]
        enemy_selected_block_pos5 = block_pos_b[enemy_r5,:]
        enemy_selected_block_status5 = goal_blues[enemy_r5,0]
        enemy_color5 = "Blue block, 3 points, "
    #Enemy selecting the blocks from our goal area (where enemy places its blocks)
    if goal_us_n5 == 0:
        goal_us_selected_distance5 = goal_us_distance_to_block_g[goal_us_r5,0]
        goal_us_selected_block_pos5 = block_pos_g[goal_us_r5,:]
        goal_us_selected_block_status5 = goal_greens[goal_us_r5,0]
        goal_us_color5 = "Green block, 1 point, "
    if goal_us_n5 == 1:
        goal_us_selected_distance5 = goal_us_distance_to_block_o[goal_us_r5,0]
        goal_us_selected_block_pos5 = block_pos_o[goal_us_r5,:]
        goal_us_selected_block_status5 = goal_oranges[goal_us_r5,0]
        goal_us_color5 = "Orange block, 2 points, "
    if goal_us_n5 == 2:
        goal_us_selected_distance5 = goal_us_distance_to_block_b[goal_us_r5,0]
        goal_us_selected_block_pos5 = block_pos_b[goal_us_r5,:]
        goal_us_selected_block_status5 = goal_blues[goal_us_r5,0]
        goal_us_color5 = "Blue block, 3 points, "

    ###The sixth weighted selection:
    #Our robot
    if n6 == 0:
        selected_distance6 = distance_to_block_g[r6,0]
        selected_block_pos6 = block_pos_g[r6,:]
        selected_block_status6 = goal_greens[r6,1]
        color6 = "Green block, 1 point, "
    if n6 == 1:
        selected_distance6 = distance_to_block_o[r6,0]
        selected_block_pos6 = block_pos_o[r6,:]
        selected_block_status6 = goal_oranges[r6,1]
        color6 = "Orange block, 2 points, "
    if n6 == 2:
        selected_distance6 = distance_to_block_b[r6,0]
        selected_block_pos6 = block_pos_b[r6,:]
        selected_block_status6 = goal_blues[r6,1]
        color6 = "Blue block, 3 points, "
    #Enemy robot
    if enemy_n6 == 0:
        enemy_selected_distance6 = enemy_distance_to_block_g[enemy_r6,0]
        enemy_selected_block_pos6 = block_pos_g[enemy_r6,:]
        enemy_selected_block_status6 = goal_greens[enemy_r6,0]
        enemy_color6 = "Green block, 1 point, "
    if enemy_n6 == 1:
        enemy_selected_distance6 = enemy_distance_to_block_o[enemy_r6,0]
        enemy_selected_block_pos6 = block_pos_o[enemy_r6,:]
        enemy_selected_block_status6 = goal_oranges[enemy_r6,0]
        enemy_color6 = "Orange block, 2 points, "
    if enemy_n6 == 2:
        enemy_selected_distance6 = enemy_distance_to_block_b[enemy_r6,0]
        enemy_selected_block_pos6 = block_pos_b[enemy_r6,:]
        enemy_selected_block_status6 = goal_blues[enemy_r6,0]
        enemy_color6 = "Blue block, 3 points, "
    #Enemy selecting the blocks from our goal area (where enemy places its blocks)
    if goal_us_n6 == 0:
        goal_us_selected_distance6 = goal_us_distance_to_block_g[goal_us_r6,0]
        goal_us_selected_block_pos6 = block_pos_g[goal_us_r6,:]
        goal_us_selected_block_status6 = goal_greens[goal_us_r6,0]
        goal_us_color6 = "Green block, 1 point, "
    if goal_us_n6 == 1:
        goal_us_selected_distance6 = goal_us_distance_to_block_o[goal_us_r6,0]
        goal_us_selected_block_pos6 = block_pos_o[goal_us_r6,:]
        goal_us_selected_block_status6 = goal_oranges[goal_us_r6,0]
        goal_us_color6 = "Orange block, 2 points, "
    if goal_us_n6 == 2:
        goal_us_selected_distance6 = goal_us_distance_to_block_b[goal_us_r6,0]
        goal_us_selected_block_pos6 = block_pos_b[goal_us_r6,:]
        goal_us_selected_block_status6 = goal_blues[goal_us_r6,0]
        goal_us_color6 = "Blue block, 3 points, "

    ###The seventh weighted selection:
    #Our robot
    if n7 == 0:
        selected_distance7 = distance_to_block_g[r7,0]
        selected_block_pos7 = block_pos_g[r7,:]
        selected_block_status7 = goal_greens[r7,1]
        color7 = "Green block, 1 point, "
    if n7 == 1:
        selected_distance7 = distance_to_block_o[r7,0]
        selected_block_pos7 = block_pos_o[r7,:]
        selected_block_status7 = goal_oranges[r7,1]
        color7 = "Orange block, 2 points, "
    if n7 == 2:
        selected_distance7 = distance_to_block_b[r7,0]
        selected_block_pos7 = block_pos_b[r7,:]
        selected_block_status7 = goal_blues[r7,1]
        color7 = "Blue block, 3 points, "
    #Enemy robot
    if enemy_n7 == 0:
        enemy_selected_distance7 = enemy_distance_to_block_g[enemy_r7,0]
        enemy_selected_block_pos7 = block_pos_g[enemy_r7,:]
        enemy_selected_block_status7 = goal_greens[enemy_r7,0]
        enemy_color7 = "Green block, 1 point, "
    if enemy_n7 == 1:
        enemy_selected_distance7 = enemy_distance_to_block_o[enemy_r7,0]
        enemy_selected_block_pos7 = block_pos_o[enemy_r7,:]
        enemy_selected_block_status7 = goal_oranges[enemy_r7,0]
        enemy_color7 = "Orange block, 2 points, "
    if enemy_n7 == 2:
        enemy_selected_distance7 = enemy_distance_to_block_b[enemy_r7,0]
        enemy_selected_block_pos7 = block_pos_b[enemy_r7,:]
        enemy_selected_block_status7 = goal_blues[enemy_r7,0]
        enemy_color7 = "Blue block, 3 points, "
    #Enemy selecting the blocks from our goal area (where enemy places its blocks)
    if goal_us_n7 == 0:
        goal_us_selected_distance7 = goal_us_distance_to_block_g[goal_us_r7,0]
        goal_us_selected_block_pos7 = block_pos_g[goal_us_r7,:]
        goal_us_selected_block_status7 = goal_greens[goal_us_r7,0]
        goal_us_color7 = "Green block, 1 point, "
    if goal_us_n7 == 1:
        goal_us_selected_distance7 = goal_us_distance_to_block_o[goal_us_r7,0]
        goal_us_selected_block_pos7 = block_pos_o[goal_us_r7,:]
        goal_us_selected_block_status7 = goal_oranges[goal_us_r7,0]
        goal_us_color7 = "Orange block, 2 points, "
    if goal_us_n7 == 2:
        goal_us_selected_distance7 = goal_us_distance_to_block_b[goal_us_r7,0]
        goal_us_selected_block_pos7 = block_pos_b[goal_us_r7,:]
        goal_us_selected_block_status7 = goal_blues[goal_us_r7,0]
        goal_us_color7 = "Blue block, 3 points, "


    enemy_distance_to_goal = np.sqrt(((x_goal_us-x_e)**2)+(y_goal_us-y_e)**2)

    if enemy_get_block == True or enemy_has_block == True:
        if goal_us_selected_block_status2 == False:
            enemy_distance_to_next_block = enemy_distance_to_goal + goal_us_selected_distance2
            enemy_selected_block = goal_us_selected_block_pos2
        else:
            if goal_us_selected_block_status3 == False:
                enemy_distance_to_next_block = enemy_distance_to_goal + goal_us_selected_distance3
                enemy_selected_block = goal_us_selected_block_pos3
            else:
                if goal_us_selected_block_status4 == False:
                    enemy_distance_to_next_block = enemy_distance_to_goal + goal_us_selected_distance4
                    enemy_selected_block = goal_us_selected_block_pos4
                else:
                    if goal_us_selected_block_status5 == False:
                        enemy_distance_to_next_block = enemy_distance_to_goal + goal_us_selected_distance5
                        enemy_selected_block = goal_us_selected_block_pos5
                    else:
                        if goal_us_selected_block_status6 == False:
                            enemy_distance_to_next_block = enemy_distance_to_goal + goal_us_selected_distance6
                            enemy_selected_block = goal_us_selected_block_pos6
                        else:
                            if goal_us_selected_block_status7 == False:
                                enemy_distance_to_next_block = enemy_distance_to_goal + goal_us_selected_distance7
                                enemy_selected_block = goal_us_selected_block_pos7
                            else:
                                enemy_distance_to_next_block = inf
    else:
        if enemy_selected_block_status1 == False:
            enemy_distance_to_next_block = enemy_selected_distance1
            enemy_selected_block = enemy_selected_block_pos1
        else:
            if enemy_selected_block_status2 == False:
                enemy_distance_to_next_block = enemy_selected_distance2
                enemy_selected_block = enemy_selected_block_pos2
            else:
                if enemy_selected_block_status3 == False:
                    enemy_distance_to_next_block = enemy_selected_distance3
                    enemy_selected_block = enemy_selected_block_pos3
                else:
                    if enemy_selected_block_status4 == False:
                        enemy_distance_to_next_block = enemy_selected_distance4
                        enemy_selected_block = enemy_selected_block_pos4
                    else:
                        if enemy_selected_block_status5 == False:
                            enemy_distance_to_next_block = enemy_selected_distance5
                            enemy_selected_block = enemy_selected_block_pos5
                        else:
                            if enemy_selected_block_status6 == False:
                                enemy_distance_to_next_block = enemy_selected_distance6
                                enemy_selected_block = enemy_selected_block_pos6
                            else:
                                if enemy_selected_block_status7 == False:
                                    enemy_distance_to_next_block = enemy_selected_distance7
                                    enemy_selected_block = enemy_selected_block_pos7
                                else:
                                    enemy_distance_to_next_block = inf
                                    enemy_selected_block = [inf, inf]

    if enemy_distance_to_next_block == inf:
        print('Enemy is in the defensive mode')
    else:
        print('Enemy is in the offensive mode')

    ### The strategy (decision tree)
    infi = np.array([inf, inf])

    if selected_block_status1 == False:
        if (selected_block_pos1 == enemy_selected_block).all():
            if selected_distance1 < enemy_distance_to_next_block:
                selected_block = selected_block_pos1
            else:
                if selected_block_status2 == False:
                    if (selected_block_pos2 == enemy_selected_block).all():
                        if selected_distance2 < enemy_distance_to_next_block:
                            selected_block = selected_block_pos2
                        else:
                            if selected_block_status3 == False:
                                if (selected_block_pos3 == enemy_selected_block).all():
                                    if selected_distance3 < enemy_distance_to_next_block:
                                        selected_block = selected_block_pos3
                                    else:
                                        if selected_block_status4 == False:
                                            if (selected_block_pos4 == enemy_selected_block).all():
                                                if selected_distance4 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos4
                                                else:
                                                    if selected_block_status5 == False:
                                                        if (selected_block_pos5 == enemy_selected_block).all():
                                                            if selected_distance5 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos5
                                                            else:
                                                                if selected_block_status6 == False:
                                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                                            selected_block = selected_block_pos6
                                                                        else:
                                                                            if selected_block_status7 == False:
                                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                                        selected_block = selected_block_pos7
                                                                                    else:
                                                                                        print('Go defensive')
                                                                                        selected_block = selected_block_pos7
                                                                                else:
                                                                                    selected_block = selected_block_pos7
                                                                            else:
                                                                                print('Go defensive')
                                                                                selected_block = infi
                                                                    else:
                                                                        selected_block = selected_block_pos6
                                                                else:
                                                                    if selected_block_status7 == False:
                                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                                selected_block = selected_block_pos7
                                                                            else:
                                                                                print('Go defensive')
                                                                                selected_block = selected_block_pos7
                                                                        else:
                                                                            selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = infi
                                                        else:
                                                            selected_block = selected_block_pos5
                                                    else:
                                                        if selected_block_status6 == False:
                                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                                if selected_distance6 < enemy_distance_to_next_block:
                                                                    selected_block = selected_block_pos6
                                                                else:
                                                                    if selected_block_status7 == False:
                                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                                selected_block = selected_block_pos7
                                                                            else:
                                                                                print('Go defensive')
                                                                                selected_block = selected_block_pos7
                                                                        else:
                                                                            selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = infi
                                                            else:
                                                                selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                            else:
                                                selected_block = selected_block_pos4
                                        else:
                                            if selected_block_status5 == False:
                                                if (selected_block_pos5 == enemy_selected_block).all():
                                                    if selected_distance5 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos5
                                                    else:
                                                        if selected_block_status6 == False:
                                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                                if selected_distance6 < enemy_distance_to_next_block:
                                                                    selected_block = selected_block_pos6
                                                                else:
                                                                    if selected_block_status7 == False:
                                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                                selected_block = selected_block_pos7
                                                                            else:
                                                                                print('Go defensive')
                                                                                selected_block = selected_block_pos7
                                                                        else:
                                                                            selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = infi
                                                            else:
                                                                selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                else:
                                                    selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                else:
                                    selected_block = selected_block_pos3
                            else:
                                if selected_block_status4 == False:
                                    if (selected_block_pos4 == enemy_selected_block).all():
                                        if selected_distance4 < enemy_distance_to_next_block:
                                            selected_block = selected_block_pos4
                                        else:
                                            if selected_block_status5 == False:
                                                if (selected_block_pos5 == enemy_selected_block).all():
                                                    if selected_distance5 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos5
                                                    else:
                                                        if selected_block_status6 == False:
                                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                                if selected_distance6 < enemy_distance_to_next_block:
                                                                    selected_block = selected_block_pos6
                                                                else:
                                                                    if selected_block_status7 == False:
                                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                                selected_block = selected_block_pos7
                                                                            else:
                                                                                print('Go defensive')
                                                                                selected_block = selected_block_pos7
                                                                        else:
                                                                            selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = infi
                                                            else:
                                                                selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                else:
                                                    selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                    else:
                                        selected_block = selected_block_pos4
                                else:
                                    if selected_block_status5 == False:
                                        if (selected_block_pos5 == enemy_selected_block).all():
                                            if selected_distance5 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                        else:
                                            selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                    else:
                        selected_block = selected_block_pos2
                else:
                    if selected_block_status3 == False:
                        if (selected_block_pos3 == enemy_selected_block).all():
                            if selected_distance3 < enemy_distance_to_next_block:
                                selected_block = selected_block_pos3
                            else:
                                if selected_block_status4 == False:
                                    if (selected_block_pos4 == enemy_selected_block).all():
                                        if selected_distance4 < enemy_distance_to_next_block:
                                            selected_block = selected_block_pos4
                                        else:
                                            if selected_block_status5 == False:
                                                if (selected_block_pos5 == enemy_selected_block).all():
                                                    if selected_distance5 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos5
                                                    else:
                                                        if selected_block_status6 == False:
                                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                                if selected_distance6 < enemy_distance_to_next_block:
                                                                    selected_block = selected_block_pos6
                                                                else:
                                                                    if selected_block_status7 == False:
                                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                                selected_block = selected_block_pos7
                                                                            else:
                                                                                print('Go defensive')
                                                                                selected_block = selected_block_pos7
                                                                        else:
                                                                            selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = infi
                                                            else:
                                                                selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                else:
                                                    selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                    else:
                                        selected_block = selected_block_pos4
                                else:
                                    if selected_block_status5 == False:
                                        if (selected_block_pos5 == enemy_selected_block).all():
                                            if selected_distance5 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                        else:
                                            selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                        else:
                            selected_block = selected_block_pos3
                    else:
                        if selected_block_status4 == False:
                            if (selected_block_pos4 == enemy_selected_block).all():
                                if selected_distance4 < enemy_distance_to_next_block:
                                    selected_block = selected_block_pos4
                                else:
                                    if selected_block_status5 == False:
                                        if (selected_block_pos5 == enemy_selected_block).all():
                                            if selected_distance5 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                        else:
                                            selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                            else:
                                selected_block = selected_block_pos4
                        else:
                            if selected_block_status5 == False:
                                if (selected_block_pos5 == enemy_selected_block).all():
                                    if selected_distance5 < enemy_distance_to_next_block:
                                        selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                                else:
                                    selected_block = selected_block_pos5
                            else:
                                if selected_block_status6 == False:
                                    if (selected_block_pos6 == enemy_selected_block).all():
                                        if selected_distance6 < enemy_distance_to_next_block:
                                            selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                                    else:
                                        selected_block = selected_block_pos6
                                else:
                                    if selected_block_status7 == False:
                                        if (selected_block_pos7 == enemy_selected_block).all():
                                            if selected_distance7 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = selected_block_pos7
                                        else:
                                            selected_block = selected_block_pos7
                                    else:
                                        print('Go defensive')
                                        selected_block = infi
        else:
            selected_block = selected_block_pos1
    else:
        if selected_block_status2 == False:
            if (selected_block_pos2 == enemy_selected_block).all():
                if selected_distance2 < enemy_distance_to_next_block:
                    selected_block = selected_block_pos2
                else:
                    if selected_block_status3 == False:
                        if (selected_block_pos3 == enemy_selected_block).all():
                            if selected_distance3 < enemy_distance_to_next_block:
                                selected_block = selected_block_pos3
                            else:
                                if selected_block_status4 == False:
                                    if (selected_block_pos4 == enemy_selected_block).all():
                                        if selected_distance4 < enemy_distance_to_next_block:
                                            selected_block = selected_block_pos4
                                        else:
                                            if selected_block_status5 == False:
                                                if (selected_block_pos5 == enemy_selected_block).all():
                                                    if selected_distance5 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos5
                                                    else:
                                                        if selected_block_status6 == False:
                                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                                if selected_distance6 < enemy_distance_to_next_block:
                                                                    selected_block = selected_block_pos6
                                                                else:
                                                                    if selected_block_status7 == False:
                                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                                selected_block = selected_block_pos7
                                                                            else:
                                                                                print('Go defensive')
                                                                                selected_block = selected_block_pos7
                                                                        else:
                                                                            selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = infi
                                                            else:
                                                                selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                else:
                                                    selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                    else:
                                        selected_block = selected_block_pos4
                                else:
                                    if selected_block_status5 == False:
                                        if (selected_block_pos5 == enemy_selected_block).all():
                                            if selected_distance5 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                        else:
                                            selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                        else:
                            selected_block = selected_block_pos3
                    else:
                        if selected_block_status4 == False:
                            if (selected_block_pos4 == enemy_selected_block).all():
                                if selected_distance4 < enemy_distance_to_next_block:
                                    selected_block = selected_block_pos4
                                else:
                                    if selected_block_status5 == False:
                                        if (selected_block_pos5 == enemy_selected_block).all():
                                            if selected_distance5 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                        else:
                                            selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                            else:
                                selected_block = selected_block_pos4
                        else:
                            if selected_block_status5 == False:
                                if (selected_block_pos5 == enemy_selected_block).all():
                                    if selected_distance5 < enemy_distance_to_next_block:
                                        selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                                else:
                                    selected_block = selected_block_pos5
                            else:
                                if selected_block_status6 == False:
                                    if (selected_block_pos6 == enemy_selected_block).all():
                                        if selected_distance6 < enemy_distance_to_next_block:
                                            selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                                    else:
                                        selected_block = selected_block_pos6
                                else:
                                    if selected_block_status7 == False:
                                        if (selected_block_pos7 == enemy_selected_block).all():
                                            if selected_distance7 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = selected_block_pos7
                                        else:
                                            selected_block = selected_block_pos7
                                    else:
                                        print('Go defensive')
                                        selected_block = infi
            else:
                selected_block = selected_block_pos2
        else:
            if selected_block_status3 == False:
                if (selected_block_pos3 == enemy_selected_block).all():
                    if selected_distance3 < enemy_distance_to_next_block:
                        selected_block = selected_block_pos3
                    else:
                        if selected_block_status4 == False:
                            if (selected_block_pos4 == enemy_selected_block).all():
                                if selected_distance4 < enemy_distance_to_next_block:
                                    selected_block = selected_block_pos4
                                else:
                                    if selected_block_status5 == False:
                                        if (selected_block_pos5 == enemy_selected_block).all():
                                            if selected_distance5 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos5
                                            else:
                                                if selected_block_status6 == False:
                                                    if (selected_block_pos6 == enemy_selected_block).all():
                                                        if selected_distance6 < enemy_distance_to_next_block:
                                                            selected_block = selected_block_pos6
                                                        else:
                                                            if selected_block_status7 == False:
                                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                                        selected_block = selected_block_pos7
                                                                    else:
                                                                        print('Go defensive')
                                                                        selected_block = selected_block_pos7
                                                                else:
                                                                    selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = infi
                                                    else:
                                                        selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                        else:
                                            selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                            else:
                                selected_block = selected_block_pos4
                        else:
                            if selected_block_status5 == False:
                                if (selected_block_pos5 == enemy_selected_block).all():
                                    if selected_distance5 < enemy_distance_to_next_block:
                                        selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                                else:
                                    selected_block = selected_block_pos5
                            else:
                                if selected_block_status6 == False:
                                    if (selected_block_pos6 == enemy_selected_block).all():
                                        if selected_distance6 < enemy_distance_to_next_block:
                                            selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                                    else:
                                        selected_block = selected_block_pos6
                                else:
                                    if selected_block_status7 == False:
                                        if (selected_block_pos7 == enemy_selected_block).all():
                                            if selected_distance7 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = selected_block_pos7
                                        else:
                                            selected_block = selected_block_pos7
                                    else:
                                        print('Go defensive')
                                        selected_block = infi
                else:
                    selected_block = selected_block_pos3
            else:
                if selected_block_status4 == False:
                    if (selected_block_pos4 == enemy_selected_block).all():
                        if selected_distance4 < enemy_distance_to_next_block:
                            selected_block = selected_block_pos4
                        else:
                            if selected_block_status5 == False:
                                if (selected_block_pos5 == enemy_selected_block).all():
                                    if selected_distance5 < enemy_distance_to_next_block:
                                        selected_block = selected_block_pos5
                                    else:
                                        if selected_block_status6 == False:
                                            if (selected_block_pos6 == enemy_selected_block).all():
                                                if selected_distance6 < enemy_distance_to_next_block:
                                                    selected_block = selected_block_pos6
                                                else:
                                                    if selected_block_status7 == False:
                                                        if (selected_block_pos7 == enemy_selected_block).all():
                                                            if selected_distance7 < enemy_distance_to_next_block:
                                                                selected_block = selected_block_pos7
                                                            else:
                                                                print('Go defensive')
                                                                selected_block = selected_block_pos7
                                                        else:
                                                            selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = infi
                                            else:
                                                selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                                else:
                                    selected_block = selected_block_pos5
                            else:
                                if selected_block_status6 == False:
                                    if (selected_block_pos6 == enemy_selected_block).all():
                                        if selected_distance6 < enemy_distance_to_next_block:
                                            selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                                    else:
                                        selected_block = selected_block_pos6
                                else:
                                    if selected_block_status7 == False:
                                        if (selected_block_pos7 == enemy_selected_block).all():
                                            if selected_distance7 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = selected_block_pos7
                                        else:
                                            selected_block = selected_block_pos7
                                    else:
                                        print('Go defensive')
                                        selected_block = infi
                    else:
                        selected_block = selected_block_pos4
                else:
                    if selected_block_status5 == False:
                        if (selected_block_pos5 == enemy_selected_block).all():
                            if selected_distance5 < enemy_distance_to_next_block:
                                selected_block = selected_block_pos5
                            else:
                                if selected_block_status6 == False:
                                    if (selected_block_pos6 == enemy_selected_block).all():
                                        if selected_distance6 < enemy_distance_to_next_block:
                                            selected_block = selected_block_pos6
                                        else:
                                            if selected_block_status7 == False:
                                                if (selected_block_pos7 == enemy_selected_block).all():
                                                    if selected_distance7 < enemy_distance_to_next_block:
                                                        selected_block = selected_block_pos7
                                                    else:
                                                        print('Go defensive')
                                                        selected_block = selected_block_pos7
                                                else:
                                                    selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = infi
                                    else:
                                        selected_block = selected_block_pos6
                                else:
                                    if selected_block_status7 == False:
                                        if (selected_block_pos7 == enemy_selected_block).all():
                                            if selected_distance7 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = selected_block_pos7
                                        else:
                                            selected_block = selected_block_pos7
                                    else:
                                        print('Go defensive')
                                        selected_block = infi
                        else:
                            selected_block = selected_block_pos5
                    else:
                        if selected_block_status6 == False:
                            if (selected_block_pos6 == enemy_selected_block).all():
                                if selected_distance6 < enemy_distance_to_next_block:
                                    selected_block = selected_block_pos6
                                else:
                                    if selected_block_status7 == False:
                                        if (selected_block_pos7 == enemy_selected_block).all():
                                            if selected_distance7 < enemy_distance_to_next_block:
                                                selected_block = selected_block_pos7
                                            else:
                                                print('Go defensive')
                                                selected_block = selected_block_pos7
                                        else:
                                            selected_block = selected_block_pos7
                                    else:
                                        print('Go defensive')
                                        selected_block = infi
                            else:
                                selected_block = selected_block_pos6
                        else:
                            if selected_block_status7 == False:
                                if (selected_block_pos7 == enemy_selected_block).all():
                                    if selected_distance7 < enemy_distance_to_next_block:
                                        selected_block = selected_block_pos7
                                    else:
                                        print('Go defensive')
                                        selected_block = selected_block_pos7
                                else:
                                    selected_block = selected_block_pos7
                            else:
                                print('Go defensive')
                                selected_block = infi


    if (selected_block == infi).all():
        ctgt = goal_center_us
        print('Go defensive')
    else:
        ctgt = selected_block

    strategy (ctgt)
