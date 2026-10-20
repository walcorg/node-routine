#!/usr/bin/env python3
"""
WALCORG TERRAFORMING CORP. - Ejemplo de maestro I2C (Raspberry Pi Zero W / shield)
para el firmware walcorg_nodo_v45.ino.

Requiere:  pip install smbus2
Recomendado en el Pi: I2C por software (dtoverlay=i2c-gpio) por el error de
clock stretching del BCM2835 en el I2C por hardware.
"""
import struct
import time
from smbus2 import SMBus, i2c_msg

BUS = 1          # con dtoverlay=i2c-gpio suele ser otro numero (p. ej. 3)
ADDR = 0x10      # 0x10..0x13 segun los solder jumpers del nodo

ESTADOS = {0: "LIBRE", 1: "OCUPADO", 2: "UVC", 3: "REPRODUCIENDO"}
ERRORES = {0: "ninguno", 1: "comando desconocido", 2: "argumento invalido",
           3: "ocupado", 4: "bloqueado por UVC activo", 5: "abortado"}
BANDERAS = ["DATOS_LISTOS", "DARK_ALERT", "XERR", "UV_ACTIVO",
            "RANGO_SENSIBLE", "INT_EXP", "REINICIO_WDT"]


class Nodo:
    def __init__(self, bus, addr=ADDR):
        self.bus, self.addr = bus, addr

    def _write(self, data):
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, list(data)))

    def comando(self, letra, *args):
        self._write([ord(letra), *args])

    def estado(self):
        w = i2c_msg.write(self.addr, [ord("?")])
        r = i2c_msg.read(self.addr, 8)
        self.bus.i2c_rdwr(w, r)
        b = list(r)
        return {
            "estado": ESTADOS.get(b[0], b[0]),
            "banderas": [n for i, n in enumerate(BANDERAS) if b[1] & (1 << i)],
            "ultimo_cmd": chr(b[2]) if b[2] else "",
            "error": ERRORES.get(b[3], b[3]),
            "fw": b[4],
            "uv_seg": b[5] | (b[6] << 8),
            "addr": hex(b[7]),
        }

    def esperar(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            e = self.estado()
            if e["estado"] in ("LIBRE", "UVC"):
                return e
            time.sleep(0.02)
        raise TimeoutError("el nodo no termino a tiempo")

    def pagina(self, n):
        """0 = ultima medicion; 1 = oscuridad; 2..13 = LED 0..11; 14 = config."""
        w = i2c_msg.write(self.addr, [ord("R"), n])
        r = i2c_msg.read(self.addr, 32)
        self.bus.i2c_rdwr(w, r)
        return list(struct.unpack("<16H", bytes(list(r))))

    # ---- operaciones de alto nivel ----
    def verificar_oscuridad(self):
        self.comando("G"); e = self.esperar()
        return "DARK_ALERT" not in e["banderas"], self.pagina(0)

    def medir_led(self, n):
        self.comando("H", n); self.esperar()
        return self.pagina(0)

    def codificar_texto(self, texto):
        """Muestra cada caracter en los 8 LEDs de datos y mide los 16 sensores."""
        resultados = []
        for ch in texto.encode("ascii"):
            self.comando("K", ch); self.esperar()
            resultados.append((chr(ch), self.pagina(0)))
        self.comando("O")
        return resultados

    def calibrar(self):
        self.comando("C"); self.esperar(timeout=30)
        return {"oscuridad": self.pagina(1),
                **{f"led{i}": self.pagina(2 + i) for i in range(12)}}

    def desinfectar(self, segundos=300):
        self.comando("E", segundos & 0xFF, segundos >> 8)

    def detener_uv(self):
        self.comando("F")

    def modo_seguro(self):
        self.comando("D")

    def parametro(self, pid, valor):
        self.comando("P", pid, valor & 0xFF, valor >> 8)


if __name__ == "__main__":
    with SMBus(BUS) as bus:
        n = Nodo(bus)
        print("Estado:", n.estado())

        oscuro, lect = n.verificar_oscuridad()
        print("Oscuridad OK" if oscuro else "ALERTA: entra luz", lect)

        print("LED 3 ->", n.medir_led(3))

        for ch, lect in n.codificar_texto("WALCORG"):
            print(ch, lect[8:])            # SENS_T0..T7 (pared receptora)

        # Desinfeccion de prueba de 60 s (solo con el nodo vacio y la tapa cerrada)
        # n.desinfectar(60)
        # while n.estado()["estado"] == "UVC":
        #     print("UV restante:", n.estado()["uv_seg"], "s"); time.sleep(5)
