def main():
    import rclpy
    from crazyflie_control.nodo_piloto import NodoPiloto

    rclpy.init()
    nodo_piloto = NodoPiloto()
    rclpy.spin(nodo_piloto)
    nodo_piloto.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
