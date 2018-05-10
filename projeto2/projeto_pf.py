#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Esta classe deve conter todas as suas implementações relevantes para seu filtro de partículas
"""

from pf import Particle, create_particles, draw_random_sample
import numpy as np
import inspercles # necessário para o a função nb_lidar que simula o laser
import math
from scipy import stats
from scipy.stats import norm


largura = 775 # largura do mapa
altura = 748  # altura do mapa


# Robo
robot = Particle(largura/2, altura/2, math.pi/4, 1.0)

# Nuvem de particulas
particulas = []

num_particulas = 300


# Os angulos em que o robo simulado vai ter sensores
angles = np.linspace(0.0, 2*math.pi, num=8, endpoint=False)

# Lista mais longa
movimentos_longos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0],
              [0,0,math.pi/12.0], [0, 0, math.pi/12.0], [0, 0, math.pi/12],[0,0,-math.pi/4],
              [-5, 0, 0],[-5,0,0], [-5,0,0], [-10,0,0],[-10,0,0], [-10,0,0],[-10,0,0],[-10,0,0],[-15,0,0],
              [0,0,-math.pi/4],[0, 10, 0], [0,10,0], [0, 10, 0], [0,10,0], [0,0,math.pi/8], [0,10,0], [0,10,0], 
              [0,10,0], [0,10,0], [0,10,0],[0,10,0],
              [0,0,-math.radians(90)],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],[math.cos(math.pi/3)*10, math.sin(math.pi/3),0],
              [math.cos(math.pi/3)*10, math.sin(math.pi/3),0]]

# Lista curta
movimentos_curtos = [[-10, -10, 0], [-10, 10, 0], [-10,0,0], [-10, 0, 0]]

movimentos_relativos = [[0, -math.pi/3],[10, 0],[10, 0], [10, 0], [10, 0],[15, 0],[15, 0],[15, 0],[0, -math.pi/2],[10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [0, -math.pi/2], 
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [0, -math.pi/2], 
                       [10,0], [0, -math.pi/4], [10,0], [10,0], [10,0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0],
                       [10,0], [10, 0], [10, 0], [10, 0], [10, 0], [10, 0]]



movimentos = movimentos_relativos


#def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas-1): e particula igual do robo
def cria_particulas(minx=0, miny=0, maxx=largura, maxy=altura, n_particulas=num_particulas):
    """
        Cria uma lista de partículas distribuídas de forma uniforme entre minx, miny, maxx e maxy
    """
        ##create_particles(pose, var_x = 50, var_y = 50, var_theta = math.pi/3, num=10):  

    #assumindo que o robo está no centro
    varx = (maxx - minx)/2
    vary = (maxy - miny)/2
    var_theta = math.pi

    return create_particles(robot.pose(), varx, vary, var_theta, n_particulas)
    
def move_particulas(particulas, movimento):

    """
        Recebe um movimento na forma [deslocamento, theta]  e o aplica a todas as partículas
        Assumindo um desvio padrão para cada um dos valores
        Esta função não precisa devolver nada, e sim alterar as partículas recebidas.
        
        Sugestão: aplicar move_relative(movimento) a cada partícula
        
        Você não precisa mover o robô. O código fornecido pelos professores fará isso
        
    """
    #valores do desvio padrão (sigma) são dados
    sigma_x = 1.5 
    sigma_y = 1.5
    sigma_theta = math.radians(1.2)  #1 a 2 graus

    media_linar = movimento[0]
    media_angular = movimento[1]



    for p in particulas:  #deslocamento linear na direção que o robo ta olhando
      normal_linear = norm.rvs(loc=media_linar, scale=sigma_x)
      normal_angular = norm.rvs(loc=media_angular, scale=sigma_theta)
      particulas = p.move_relative([normal_linear,normal_angular])

    return particulas
    
def leituras_laser_evidencias(robot, particulas):
    """
        Realiza leituras simuladas do laser para o robo e as particulas
        Depois incorpora a evidência calculando
        P(H|D) para todas as particulas
        Lembre-se de que a formula $P(z_t | x_t) = \alpha \prod_{j}^M{e^{\frac{-(z_j - \hat{z_j})}{2\sigma^2}}}$ 
        responde somente P(Hi|D), em que H é a hi
        
        Esta função não precisa retornar nada, mas as partículas precisa ter o seu w recalculado. 
        
        Você vai precisar calcular para o robo
        
    """
    # alfa = 1/soma

    #leitura do robo:
    leitura_robo = inspercles.nb_lidar(robot, angles)

    sigma = 9.8  #usar de 4 a 7 cm
    alfa = 0
    soma_alfa = 0
    for i in range (len(particulas)):
      #lista com a leitura em cada angulo da partica
      leitura_particula = inspercles.nb_lidar(particulas[i], angles)
      probabilidade =0
      
      #com chapeu é real (robo) e sem chapeu (particula)
      for b in angles:  #angles = 8
        zrj =  leitura_robo[b]     #parte real robo
        zj = leitura_particula[b]  
         
        #mudar o num de particular p maior do que 500

        #salva em w da particula da vez que é particulas[i]
        probabs = norm.pdf(zj, loc=zrj, scale=sigma)
        probabilidade += probabs
        #probabilidade += math.exp(-((zj - zrj)/2*sigma**2))
      
      particulas[i].w = probabilidade 

    for c in particulas:
      soma_alfa += c.w

    alfa = 1/soma_alfa

    for d in particulas:
      d.w *= alfa
    #multiplicar todos os angulos x alfa (isso é a probabilidade)

  # Voce vai precisar calcular a leitura para cada particula usando inspercles.nb_lidar e depois atualizar as probabilidades


    
    
def reamostrar(particulas, n_particulas = num_particulas):
    """
        Reamostra as partículas devolvendo novas particulas sorteadas
        de acordo com a probabilidade e deslocadas de acordo com uma variação normal    
        
        O notebook como_sortear tem dicas que podem ser úteis
        
        Depois de reamostradas todas as partículas precisam novamente ser deixadas com probabilidade igual
        
        Use 1/n ou 1, não importa desde que seja a mesma
    """
#######
    sigma_x = 4  
    sigma_y = 4
    sigma_theta = math.radians(12)  #
#     media = particulas[0]  ##??
#     p = norm.rvs(loc=media, scale=sigma)
#     particulas_exp = [[p.x, p.y, p.theta] for p in particulas]

    particulas_pesos = [p.w for p in particulas]
    
    novas_particulas = draw_random_sample(particulas, particulas_pesos, n_particulas)

    ##aplicar variacao normal em x,y,theta de cada p

    for p in novas_particulas:  #deslocamento linear na direção que o robo ta olhando
      normal_x = norm.rvs(loc=0, scale=sigma_x)
      normal_y = norm.rvs(loc=0, scale=sigma_y)
      normal_angular = norm.rvs(loc=0, scale=sigma_theta)
      p.x += normal_x 
      p.y += normal_y
      p.theta += normal_angular

    for p in novas_particulas:
      p.w = 1

      
   #####  norm.rvs(media, sigma)
    return novas_particulas


    



