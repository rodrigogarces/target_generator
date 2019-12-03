# Target Generator
> Sistema de geração de targets para exploração autônoma de robôs móveis.

Este pacote ros propricia a navegação autônoma e mapeamento de robôs móveis em abientes desconhecidos. Ao receber um occupancy grid do node /SLAM, que faz o mapemanento e localização simultaneos, trasnforma o mapa em uma superficie de função objetivo e executa algorimos de otimização para busca os pontos do mapa mais interessantes para o robôs. Ao encontrar o melhor ponto este é enviado para o robô, e então o robô navega até lá.

Sistema testado em simulação usando o simulador robótico [V-REP](http://www.coppeliarobotics.com/). As cenas usadas estão disponiveis no diretório ./scenes/. Necessário a instação dos pacotes [navigation](http://wiki.ros.org/navigation), [move_base](http://wiki.ros.org/move_base), [youbot_navigation](http://wiki.ros.org/youbot_navigation) e [gmapping](http://wiki.ros.org/gmapping).

![](../header.png)

## Execução

Inicie o ros:

```sh
roscore
```

Configure o parâmetro use_sim_time como true ().

```sh
rosparam set use_sim_time true
```

Inicie nodes necessários dos pacotes gmapping, ros navigation e youbot_navigation:

```sh
roslaunch target_generator target_generator.launch
```
Inicie o node gerador de targets:

```sh
rosrun target_generator target_generator
```

Abra o V-REP e inicie a simulação.

## Compilação

Clone o projeto do github na pasta src de seu ros workspace:

```sh
cd (your_ros_ws)/src
git clone https://github.com/yourname/github-link
```
Compile (Recomendo usar [Catkin Command Line Tools](http://mcs.une.edu.au/doc/python-catkin_tools-doc/html/)):

```sh
cd ..
catkin build
```

## Meta

Distribuído sob a licença XYZ. Veja `LICENSE` para mais informações.

[https://github.com/yourname/github-link](https://github.com/othonalberto/)

###### Contribuidores:

Raphael Gomes – https://github.com/raphaelgoms / raphaelgoms@gmail.com

## Contributing

1. Faça o _fork_ do projeto (<https://github.com/yourname/yourproject/fork>)
2. Crie uma _branch_ para sua modificação (`git checkout -b feature/fooBar`)
3. Faça o _commit_ (`git commit -am 'Add some fooBar'`)
4. _Push_ (`git push origin feature/fooBar`)
5. Crie um novo _Pull Request_

