#!/usr/bin/env python3
"""
unir_datos.py - Une los archivos CSV de las tres configuraciones en uno solo
para facilitar la comparación.

Lee:
  - datos_crudo_*.csv
  - datos_fir_*.csv
  - datos_kalman_*.csv

Genera:
  - datos_comparativa_todos_YYYYMMDD_HHMMSS.csv

No requiere librerías externas (usa solo la librería estándar de Python).

Autor: Laboratorio 3 - Procesamiento de Señales
"""

import os
import csv
import glob
from datetime import datetime


CARPETA_DATOS = os.path.dirname(os.path.abspath(__file__))


def encontrar_archivo(patron):
    """Busca el archivo más reciente que coincida con el patrón."""
    archivos = glob.glob(os.path.join(CARPETA_DATOS, patron))
    if not archivos:
        return None
    # Ordenar por fecha de modificación (el más reciente primero)
    archivos.sort(key=os.path.getmtime, reverse=True)
    return archivos[0]


def cargar_csv(ruta_archivo, nombre_configuracion):
    """Carga un CSV y agrega/actualiza la columna 'configuracion'."""
    filas = []
    with open(ruta_archivo, 'r', newline='') as f:
        lector = csv.DictReader(f)
        columnas_originales = lector.fieldnames
        for fila in lector:
            fila['configuracion'] = nombre_configuracion
            filas.append(fila)
    return filas, columnas_originales


def main():
    print("=" * 60)
    print("UNIÓN DE DATOS DE TRAYECTORIA PARA COMPARACIÓN")
    print("=" * 60)

    # Buscar los archivos más recientes de cada configuración
    archivo_crudo = encontrar_archivo("datos_crudo_*.csv")
    archivo_fir   = encontrar_archivo("datos_fir_*.csv")
    archivo_kalman = encontrar_archivo("datos_kalman_*.csv")

    print(f"\nArchivos encontrados:")
    print(f"  Crudo : {archivo_crudo if archivo_crudo else 'NO ENCONTRADO'}")
    print(f"  FIR   : {archivo_fir if archivo_fir else 'NO ENCONTRADO'}")
    print(f"  Kalman: {archivo_kalman if archivo_kalman else 'NO ENCONTRADO'}")

    todas_las_filas = []
    columnas_union = set()

    if archivo_crudo:
        filas_crudo, cols_crudo = cargar_csv(archivo_crudo, "crudo")
        todas_las_filas.extend(filas_crudo)
        columnas_union.update(cols_crudo)
        print(f"  -> Cargado: {len(filas_crudo)} filas de CRUDOS")

    if archivo_fir:
        filas_fir, cols_fir = cargar_csv(archivo_fir, "fir")
        todas_las_filas.extend(filas_fir)
        columnas_union.update(cols_fir)
        print(f"  -> Cargado: {len(filas_fir)} filas de FIR")

    if archivo_kalman:
        filas_kalman, cols_kalman = cargar_csv(archivo_kalman, "kalman")
        todas_las_filas.extend(filas_kalman)
        columnas_union.update(cols_kalman)
        print(f"  -> Cargado: {len(filas_kalman)} filas de KALMAN")

    if not todas_las_filas:
        print("\n❌ No se encontraron archivos de datos para unir.")
        return

    # Definir orden de columnas preferido
    columnas_preferidas = [
        'configuracion', 't_ms', 'seq',
        'enc_l_crudo', 'enc_r_crudo', 'enc_l_fir', 'enc_r_fir',
        'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps',
        'kal_x', 'kal_y', 'kal_theta', 'kal_v', 'kal_w', 'kal_quieto'
    ]

    # Construir orden final: primero las preferidas que existan, luego el resto
    columnas_finales = [c for c in columnas_preferidas if c in columnas_union]
    for c in sorted(columnas_union):
        if c not in columnas_finales:
            columnas_finales.append(c)

    # Ordenar por tiempo (t_ms) para que quede cronológico
    def clave_orden(fila):
        try:
            return int(fila.get('t_ms', 0))
        except (ValueError, TypeError):
            return 0

    todas_las_filas.sort(key=clave_orden)

    # Nombre del archivo de salida con timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    archivo_salida = os.path.join(CARPETA_DATOS, f'datos_comparativa_todos_{timestamp}.csv')

    # Guardar CSV unido
    with open(archivo_salida, 'w', newline='') as f:
        escritor = csv.DictWriter(f, fieldnames=columnas_finales, extrasaction='ignore')
        escritor.writeheader()
        for fila in todas_las_filas:
            escritor.writerow(fila)

    print(f"\n✅ Archivo unido guardado en:")
    print(f"   {archivo_salida}")
    print(f"   Total filas: {len(todas_las_filas)}")
    print(f"   Columnas: {len(columnas_finales)}")

    # Contar filas por configuración
    conteo = {}
    for fila in todas_las_filas:
        cfg = fila.get('configuracion', 'desconocido')
        conteo[cfg] = conteo.get(cfg, 0) + 1

    print(f"\nDistribución por configuración:")
    for cfg, cnt in sorted(conteo.items()):
        print(f"  {cfg}: {cnt} filas")

    # Mostrar resumen de posición final por configuración (usando kal_x, kal_y)
    print(f"\n{'='*60}")
    print("RESUMEN DE POSICIÓN FINAL POR CONFIGURACIÓN")
    print(f"{'='*60}")
    for config in ['crudo', 'fir', 'kalman']:
        filas_cfg = [f for f in todas_las_filas if f.get('configuracion') == config]
        if filas_cfg:
            fila_final = filas_cfg[-1]
            try:
                x = float(fila_final.get('kal_x', 0))
                y = float(fila_final.get('kal_y', 0))
                theta = float(fila_final.get('kal_theta', 0))
                theta_deg = theta * 180 / 3.14159
                dist = (x**2 + y**2)**0.5
                print(f"\n{config.upper()}:")
                print(f"  X final = {x:.4f} m")
                print(f"  Y final = {y:.4f} m")
                print(f"  Theta final = {theta:.4f} rad ({theta_deg:.1f}°)")
                print(f"  Distancia total = {dist:.4f} m")
                print(f"  Muestras = {len(filas_cfg)}")
            except (ValueError, TypeError) as e:
                print(f"\n{config.upper()}: error al leer valores finales ({e})")


if __name__ == '__main__':
    main()