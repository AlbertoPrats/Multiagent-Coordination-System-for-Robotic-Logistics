import time
import random
import socket
import math

import rclpy

import mov2 as mov
import ControlABB

from ros_gz_interfaces.srv import SetEntityPose

terminado=True

rclpy.init()
IP_ABB="192.168.137.1"
# --- CONFIGURACIÓN DEL MUNDO ---

# Destinos predefinidos en el mundo con sus coordenadas (x, y, z) y orientación (w)
DESTINOS = {
    'despensa_cebolla': [{'x': -5.5, 'y': 0.5, 'z': 0.0, 'w': 1.0}],
    'despensa_tomate':  [{'x': -5.5, 'y': -0.5, 'z': 0.0, 'w': 1.0}],
    'tabla_corte':      [{'x': -1.0, 'y': 3.0, 'z': 0.0, 'w': 1.0}, {'x': -1.0, 'y': -3.0, 'z': 0.0, 'w': 1.0}],
    'encimera':         [{'x': 3.0, 'y': 0.8, 'z': -0.8509035, 'w': 0.525322}, {'x': 3.0, 'y': -0.8, 'z': 0.8509035, 'w': 0.525322}],
    'entrega':          [{'x': 7.7, 'y': 0.75, 'z': -0.8509035, 'w': 0.525322}, {'x': 7.0, 'y': -0.75, 'z': 0.8509035, 'w': 0.525322}]
}

# Inventario inicial de ingredientes y platos disponibles
INVENTARIO = {
            "lechugas": ["lechuga_1", "lechuga_2", "lechuga_3"],
            "tomates":  ["tomate_1", "tomate_2", "tomate_3"],
            "cebollas": ["cebolla_1", "cebolla_2", "cebolla_3"],
            "platos":   ["plato_1", "plato_2", "plato_3"]
        }

# Posiciones iniciales de los robots (en realidad se sobreeescriben con el valor dentro de mov2)
origenesRobots = {
    'robot0': {'x': 0.0, 'y': 0.0, 'w': 1.0},
    'robot1': {'x': 0.0, 'y': 0.0, 'w': 1.0},
}

# Recetas con tareas desglosadas en pasos, cada paso es un diccionario con información sobre el tipo de tarea, posiciones involucradas, tipo de teletransporte y el ingrediente/plato asociado
RECETAS = {
    'sopa_cebolla': [
        {'tipo': 'transporte', 'posi': 'despensa_cebolla', 'posf': 'tabla_corte', 'tp': 'seguimiento', 'ingrediente':'cebollas'},
        {'tipo': 'operacion',  'pos': 'tabla_corte', 'accion': 'cortar', 'tp': [-1.2, 0.0, 0.98]},
        {'tipo': 'operacion', 'pos': 'encimera', 'accion': 'emplatar', 'tp': [0.74 , 0.4, 1.05]},
        {'tipo': 'transporte', 'posi': 'encimera', 'posf': 'entrega', 'tp': 'seguimiento_combo', 'ingrediente': ['cebollas', 'platos']}
    ],
    'sopa_tomate': [
        {'tipo': 'transporte', 'posi': 'despensa_tomate', 'posf': 'tabla_corte', 'tp': 'seguimiento', 'ingrediente':'tomates'},
        {'tipo': 'operacion',  'pos': 'tabla_corte', 'accion': 'cortar', 'tp': [-1.2, 0.0, 0.98]},
        {'tipo': 'operacion', 'pos': 'encimera', 'accion': 'emplatar', 'tp': [0.74 , -0.4, 1.05]},
        {'tipo': 'transporte', 'posi': 'encimera', 'posf': 'entrega', 'tp': 'seguimiento_combo', 'ingrediente': ['tomates', 'platos']}
    ]
}


cola_pedidos_activos = []

# --- ESTADO DE LOS ROBOTS ---

# Cada robot tiene un diccionario con su estado actual, incluyendo si está libre o ocupado, qué tipo de tareas puede realizar, su destino actual, el pedido que está atendiendo, y la conexión con ROS o RobotStudio según corresponda
robots = {
    'robot_0': {'id':0, 'libre': True, 'tipo': 'movil', 'destino': None, 'puede': ['transporte'], 'estado_interno': 'IDLE', 'nodo_Ros': mov.RobotCommander('robot_0',origenesRobots['robot0']), 'pedido_id': None},
    'robot_1': {'id':1, 'libre': True, 'tipo': 'movil', 'destino': None, 'puede': ['transporte'], 'estado_interno': 'IDLE', 'nodo_Ros': mov.RobotCommander('robot_1',origenesRobots['robot1']), 'pedido_id': None},
    'brazo':   {'id':2, 'libre': True, 'tipo': 'brazo', 'destino': None, 'puede': ['operacion'],  'estado_interno': 'IDLE', 'puerto_socket':5500, 'socket': None, 'nodo_Ros': ControlABB.JointPublisher('irb120_controller'), 'pedido_id': None, 'Tarea_finalizada':True }
}

# --- FUNCIONES DE COMUNICACION CON ROS ---

def conectar_brazo_ABB(r_id):
    """Establece la conexión con RobotStudio para controlar el brazo ABB"""
    robots[r_id]['socket'] = ControlABB.conectarSocket(IP_ABB, robots[r_id]['puerto_socket'])

def checkear_estado_robot(r_id):
    """Lee el topic de ros para comprobar si el robot ha llegado al destino"""
    
    rclpy.spin_once(robots[r_id]['nodo_Ros'], timeout_sec=0.1)
    if robots[r_id]['tipo'] == 'movil':
        return robots[r_id]['nodo_Ros'].check_goal()
    
    elif robots[r_id]['tipo'] == 'brazo':
        return robots[r_id]['Tarea_finalizada']

    
def mandar_robot_a_destino(r_id, destino):
    """Publica en el topic de ROS de los robots para mandar al robot a un destino"""
    robots[r_id]['nodo_Ros'].create_goal_pose(destino)
    print(f"bip bop - Mandando {r_id} a X={destino['x']}, Y={destino['y']}...")

def empezar_accion_brazo(r_id, accion):
    """Manda la señal a RobotStudio para que empiece el movimiento descrito"""
    robots[r_id]['socket'].send(accion.encode())
    robots[r_id]['Tarea_finalizada'] = False
    print(f"{accion} enviada a {r_id} ")

def checkear_brazo(r_id):
    """Lee la respuesta de RobotStudio para saber si el brazo ha terminado su acción"""
    respuesta_raw = robots[r_id]['socket'].recv(1024).decode()

    # Separamos por saltos de línea para obtener cada mensaje individual
    mensajes = respuesta_raw.strip().split('\n')
    for mensaje in mensajes:
        # Limpiamos espacios o caracteres extraños por seguridad
        mensaje = mensaje.strip()
        
        if not mensaje: 
            continue # Saltamos si hay una línea vacía
            
        if mensaje == "terminado":
            robots[r_id]['Tarea_finalizada'] = True
        else:
            try:
                # Separamos por comas el mensaje individual
                partes = mensaje.split(',')
                respuesta_int = [float(val) for val in partes]
                
                # Enviamos al nodo de ROS
                robots[r_id]['nodo_Ros'].mover(respuesta_int)
                
            except ValueError as e:
                print(f"Error procesando el mensaje '{mensaje}': {e}")


def teletransportar(r_id, destino, ingrediente, plato = None):
    """Función de teletransporte instantáneo para simular el movimiento del robot"""
    robot_node = robots[r_id]['nodo_Ros']
    mensaje = f"Teletransportando {ingrediente} y {plato} a {destino}..." if (destino == 'seguimiento_combo') else f"Teletransportando {ingrediente} a {destino}..."
    print(mensaje)

    # Para destinos de seguimiento, el ingrediente se teletransporta a la posición actual del robot para simular que lo lleva consigo
    # en otros casos, se teletransporta directamente al destino final

    # Si el nodo no tiene el cliente creado, lo creamos al vuelo
    if not hasattr(robot_node, 'gz_client'):
        robot_node.gz_client = robot_node.create_client(SetEntityPose, '/world/cocina_robotica_recreada/set_pose')
        print("Creando cliente")

    if destino == 'seguimiento' or destino == 'seguimiento_combo':

        # Teletransportamos el ingrediente a la posición actual del robot
        robot_pos = robots[r_id]['nodo_Ros'].get_current_pos()

        req = SetEntityPose.Request()
        req.entity.name = ingrediente
        req.pose.position.x = float(robot_pos[0])
        req.pose.position.y = float(robot_pos[1])
        req.pose.position.z = float(0.55)
        req.pose.orientation.w = 1.0
        robot_node.gz_client.call_async(req)
        # el client call async es no bloqueante, así que si hay plato también se teletransporta inmediatamente después sin esperar a que el ingrediente se haya "teletransportado"

        if destino == 'seguimiento_combo' and plato:

            # Teletransportamos el plato a la posición actual del robot
            req_plato = SetEntityPose.Request()
            req_plato.entity.name = plato
            req_plato.pose.position.x = float(robot_pos[0])
            req_plato.pose.position.y = float(robot_pos[1])
            req_plato.pose.position.z = float(0.5)
            req_plato.pose.orientation.w = 1.0
            robot_node.gz_client.call_async(req_plato)
    else:
        # Teletransportamos el ingrediente al destino estático
        req = SetEntityPose.Request()
        req.entity.name = ingrediente
        req.pose.position.x = float(destino[0])
        req.pose.position.y = float(destino[1])
        req.pose.position.z = float(destino[2]+0.1)
        req.pose.orientation.w = 1.0
        robot_node.gz_client.call_async(req)

        if plato:
            # Teletransportamos el plato al destino estático
            req_plato = SetEntityPose.Request()
            req_plato.entity.name = plato
            req_plato.pose.position.x = float(destino[0])
            req_plato.pose.position.y = float(destino[1])
            req_plato.pose.position.z = float(destino[2])
            req_plato.pose.orientation.w = 1.0
            robot_node.gz_client.call_async(req_plato)

def gestionar_tareas():
    """Revisa los pedidos activos y asigna tareas a los robots disponibles"""
    for pedido in cola_pedidos_activos:
        # Obtenemos el paso actual del pedido para saber qué tarea necesita realizar
        pasos = RECETAS[pedido['nombre']]
        idx = pedido['paso_actual']

        if pedido.get('paso_bloqueado', False):
            continue

        # Si hemos terminado todos los pasos del pedido, lo marcamos como completado y liberamos recursos
        if idx >= len(pasos):
            print(f"Pedido {pedido['id']}: {pedido['nombre']} COMPLETADO.")
            cola_pedidos_activos.remove(pedido)
            robot = next((r for r in robots.values() if r['pedido_id'] == pedido['id']), None) # el robot que haya estado haciendo la última tarea de este pedido
            destino_terminado = [1, 3, 0.15]    # posición escondida para ocultar el pedido "entregado"
            INVENTARIO['platos'].append(pedido['plato']) # Devolvemos el plato al inventario para reutilizarlo
            if pedido['ingrediente'] == 'cebolla_1' or pedido['ingrediente'] == 'cebolla_2' or pedido['ingrediente'] == 'cebolla_3':
                INVENTARIO['cebollas'].append(pedido['ingrediente']) # Devolvemos el ingrediente al inventario para reutilizarlo
            elif pedido['ingrediente'] == 'tomate_1' or pedido['ingrediente'] == 'tomate_2' or pedido['ingrediente'] == 'tomate_3':
                INVENTARIO['tomates'].append(pedido['ingrediente']) # Devolvemos el ingrediente al inventario para reutilizarlo
            elif pedido['ingrediente'] == 'lechuga_1' or pedido['ingrediente'] == 'lechuga_2' or pedido['ingrediente'] == 'lechuga_3':
                INVENTARIO['lechugas'].append(pedido['ingrediente']) # Devolvemos el ingrediente al inventario para reutilizarlo
            
            if(robot==None):
                robot='robot_0'
            # teletransportamos el plato "entregado" a una posición escondida para simular que el pedido ha sido entregado y ya no está en el sistema
            teletransportar(robot, destino_terminado, pedido['ingrediente'], pedido['plato']) # Teletransportamos el plato "entregado" a una posición escondida
            continue

        paso = pasos[idx]
        
        # Buscar un robot disponible y capaz
        for r_id, r_info in robots.items():
            if r_info['libre'] and paso['tipo'] in r_info['puede']:
                r_info['libre'] = False
                r_info['pedido_id'] = pedido['id']
                pedido['paso_bloqueado'] = True
                
                if paso['tipo'] == 'transporte':
                    # Iniciamos la primera fase: Recoger
                    r_info['estado_interno'] = 'YENDO_A_RECOGER'
                    r_info['destino_final_paso'] = paso['posf'] # Guardamos a dónde irá después

                    print(f"🚚 Pedido {pedido['id']}: {r_id} asignado para recoger en {paso['posi']}")
                    # Si el destino tiene una única posición, se asigna esa, si tiene varias (como la tabla de corte o la encimera), se asigna una según el ID del robot 
                    # ya que en el mapa el primer robot va por la izquierda y el segundo por la derecha, así evitamos que ambos robots vayan al mismo punto y se bloqueen entre ellos
                    if len(DESTINOS[paso['posi']]) == 1:
                        destino_inmediato = DESTINOS[paso['posi']][0]
                    else:
                        destino_inmediato = DESTINOS[paso['posi']][r_info['id']]
                    mandar_robot_a_destino(r_id, destino_inmediato)

                elif paso['tipo'] == 'operacion':
                    # Tarea de operación (brazo)
                    r_info['estado_interno'] = 'OPERANDO'
                    r_info['libre'] = False
                    destino_inmediato = DESTINOS[paso['pos']]
                    destino_tp = paso['tp']
                    ingrediente = pedido['ingrediente']

                    print(f"🤖 Pedido {pedido['id']}: {r_id} iniciando {paso['accion']} en {paso['pos']}")
                    # Para saber si el se debe teletransportar con el plato o sin él
                    if paso['pos'] == 'encimera':
                        teletransportar(r_id, destino_tp, ingrediente, pedido['plato'])
                    else:
                        teletransportar(r_id, destino_tp, ingrediente)
                    empezar_accion_brazo(r_id, paso['accion'])

                # Guardamos el destino inmediato para que el robot lo tenga presente durante la simulación del progreso
                r_info['destino'] = destino_inmediato
                r_info['libre']=False
                break

def simular_progreso_robots():
    """Simula el progreso de los robots"""
    for r_id, r_info in robots.items():
        # Si el robot está libre, no hay nada que simular
        if r_info['libre']: continue

        # Si el robot ha llegado a su destino o ha terminado su operación, avanzamos en la lógica para asignarle la siguiente tarea o marcar el pedido como completado
        if checkear_estado_robot(r_id):
            pid = r_info['pedido_id']
            pedido = next((p for p in cola_pedidos_activos if p['id'] == pid), None)
            if r_info['estado_interno'] == 'YENDO_A_RECOGER':
                # Fase 1 terminada, ahora ir a dejar
                print(f"📦 Pedido {pedido['id']}: {r_id} ha recogido el ingrediente. Ahora va a {r_info['destino_final_paso']}")
                if "despensa" in r_info['destino_final_paso']:
                    mandar_robot_a_destino(r_id, DESTINOS[r_info['destino_final_paso']])
                else:
                    mandar_robot_a_destino(r_id, DESTINOS[r_info['destino_final_paso']][r_info['id']])
                
                # Actualizamos el estado interno para reflejar que ahora va a dejar el ingrediente en su destino final
                r_info['estado_interno'] = 'YENDO_A_DEJAR'
                nuevo_destino = DESTINOS[r_info['destino_final_paso']]
                r_info['destino'] = nuevo_destino
                r_info['libre']=False
                
                # Para simular que el robot lleva el ingrediente consigo, teletransportamos el ingrediente a la posición actual del robot
                destino_tp = RECETAS[pedido['nombre']][pedido['paso_actual']]['tp']
                #pedido['paso']['tp']
                ingrediente = pedido['ingrediente']
                plato = pedido['plato']
                teletransportar(r_id, destino_tp, ingrediente, plato)

                
            elif r_info['estado_interno'] in ['YENDO_A_DEJAR', 'OPERANDO']:
                # Tarea totalmente terminada
                print(f"✨ Pedido {pedido['id']}: {r_id} ha finalizado su tarea actual.")
                r_info['libre'] = True
                r_info['estado_interno'] = 'IDLE'
                r_info['destino'] = None
                r_info['pedido_id'] = None
        
                if pedido:
                    pedido['paso_bloqueado'] = False # Desbloqueamos el pedido para que pueda avanzar al siguiente paso
                    pedido['paso_actual'] += 1
                


# --- BUCLE PRINCIPAL ---

def loop_principal():
    print("=== OVERCOOKED LOGIC ENGINE RUNNING ===")
    for r_id, r_info in robots.items():
            if r_info['tipo'] == 'brazo':
               conectar_brazo_ABB(r_id)
    contador_pedidos = 1
    while True:
        for r_id, r_info in robots.items():
           if r_info['tipo'] == 'brazo' and not robots[r_id]['libre']:
               checkear_brazo('brazo')
        # Intentar generar un pedido si no hay muchos en cola
        if len(cola_pedidos_activos) < 2 and random.random() < 0.8:
            id_str = f"{contador_pedidos:03d}"
            if contador_pedidos%2==1:
                nombre = 'sopa_cebolla'
                cola_pedidos_activos.append({'id': id_str, 'nombre': nombre, 'paso_actual': 0, 'ingrediente': INVENTARIO['cebollas'].pop(0), 'plato': INVENTARIO['platos'].pop(0)})

            else:
                nombre = 'sopa_tomate'
                cola_pedidos_activos.append({'id': id_str, 'nombre': nombre, 'paso_actual': 0, 'ingrediente': INVENTARIO['tomates'].pop(0), 'plato': INVENTARIO['platos'].pop(0)})
            print(f"\n[!] Nuevo pedido en cocina: {id_str} - {nombre}")
            contador_pedidos += 1

        simular_progreso_robots()
        gestionar_tareas()
        #input("")
    
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        loop_principal()
    except KeyboardInterrupt:
        pass