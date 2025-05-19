import tkinter as tk
from tkinter import messagebox, filedialog

def obtener_binario(valor, bits, es_constante=False):
    try:
        if es_constante:  # Manejo de constantes inmediatas
            num = int(valor.replace("#", ""))
            return format(num & ((1 << bits) - 1), f'0{bits}b')
        elif valor.startswith("$"):  # Manejo de registros
            return format(int(valor[1:]), f'0{bits}b')
        else:
            raise ValueError(f"Valor '{valor}' inválido")
    except ValueError:
        raise ValueError(f"Valor '{valor}' inválido")

def convertir_a_binario(instruccion):
    try:
        instruccion = instruccion.replace(",", "").strip()
        partes = instruccion.split()
        
        if not partes:
            raise ValueError("Instrucción vacía")
        
        opcode_dict = {
            "j": "000010", "jal": "000011", "sw": "101011", "lw": "100011", "addi": "001000", "beq": "000100",
            "andi": "001100", "ori": "001101", "xori": "001110", "slti": "001010", "bne": "000101"
        }
        funct_dict = {
            "or": "100101", "add": "100000", "sub": "100010",
            "and": "100100", "slt": "101010", "nop": "000000"
        }
        special = "000000"
        functionOp = partes[0].lower()

        if functionOp in funct_dict:  # Tipo R
            if len(partes) < 4:
                raise ValueError("Instrucción tipo R mal formada")
            rd, rs, rt = obtener_binario(partes[1], 5), obtener_binario(partes[2], 5), obtener_binario(partes[3], 5)
            return f"{special}{rs}{rt}{rd}00000{funct_dict[functionOp]}"
        
        elif functionOp in opcode_dict:  # Tipo I y J
            opcode = opcode_dict[functionOp]

            if functionOp in ["j", "jal"]:  # Tipo J con `jal`
                if len(partes) < 2:
                    raise ValueError("Instrucción tipo J mal formada")
                return f"{opcode}{format(int(partes[1][1:]), '026b')}"

            if functionOp in ["addi", "andi", "ori", "xori", "slti"]:  # Tipo I con inmediato
                if len(partes) < 4:
                    raise ValueError(f"Instrucción tipo I mal formada ({functionOp})")
                rs, rt = obtener_binario(partes[2], 5), obtener_binario(partes[1], 5)
                inmediato = obtener_binario(partes[3], 16, es_constante=True)
                return f"{opcode}{rs}{rt}{inmediato}"

            if functionOp in ["beq", "bne"]:  # Comparaciones con desvío
                if len(partes) < 4:
                    raise ValueError(f"Instrucción tipo I mal formada ({functionOp})")
                rs, rt = obtener_binario(partes[1], 5), obtener_binario(partes[2], 5)
                desplazamiento = obtener_binario(partes[3], 16, es_constante=True)
                return f"{opcode}{rs}{rt}{desplazamiento}"

            if functionOp in ["sw", "lw"]:  # Tipo I con desplazamiento
                if len(partes) < 4:
                    raise ValueError(f"Instrucción tipo I mal formada ({functionOp})")
                rs, rt = obtener_binario(partes[2], 5), obtener_binario(partes[1], 5)
                desplazamiento = obtener_binario(partes[3], 16, es_constante=True)
                return f"{opcode}{rs}{rt}{desplazamiento}"

        raise ValueError(f"Operación '{functionOp}' no reconocida")

    except ValueError as e:
        return f"Error: {e}"

def procesar_entrada_manual():
    instruccion = entrada_texto.get()
    resultado = convertir_a_binario(instruccion)
    if resultado:
        etiqueta_resultado.config(text=f"Binario:\n{resultado}")
        guardar_en_archivo(resultado)
    else:
        messagebox.showerror("Error", "Formato de instrucción incorrecto.")

def procesar_archivo():
    archivo_seleccionado = filedialog.askopenfilename(filetypes=[("Archivos de texto", "*.txt")])
    if not archivo_seleccionado:
        return
    
    instrucciones_binarias = []
    try:
        with open(archivo_seleccionado, "r") as archivo:
            for linea in archivo:
                linea = linea.strip()
                if linea:
                    resultado = convertir_a_binario(linea)
                    instrucciones_binarias.append(resultado if resultado else f"Error en la instrucción: {linea}")
        
        guardar_en_archivo('\n'.join(instrucciones_binarias))
        messagebox.showinfo("Éxito", "Archivo procesado correctamente.")
    except Exception as e:
        messagebox.showerror("Error", f"No se pudo procesar el archivo: {e}")

def guardar_en_archivo(contenido):
    if not ruta_archivo.get():
        messagebox.showerror("Error", "Seleccione una ruta para guardar el archivo.")
        return
    
    with open(ruta_archivo.get(), "w") as archivo:
        archivo.write(contenido + "\n")

def seleccionar_archivo_salida():
    archivo_seleccionado = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Archivos de Texto", "*.txt")])
    if archivo_seleccionado:
        ruta_archivo.set(archivo_seleccionado)

# Crear la ventana principal
ventana = tk.Tk()
ventana.title("Conversor de Instrucciones MIPS a Binario")
ventana.geometry("500x400")
ventana.configure(bg="#2C3E50")

ruta_archivo = tk.StringVar()

# Widgets con colores y estilos
etiqueta = tk.Label(ventana, text="Ingrese la instrucción:", bg="#2C3E50", fg="white", font=("Arial", 12))
etiqueta.pack(pady=5)

entrada_texto = tk.Entry(ventana, width=50, font=("Arial", 12), bg="#ECF0F1", fg="#2C3E50")
entrada_texto.pack(pady=5)

boton_convertir = tk.Button(ventana, text="Convertir", command=procesar_entrada_manual, bg="#3498DB", fg="white", font=("Arial", 12), padx=10, pady=5)
boton_convertir.pack(pady=5)

etiqueta_resultado = tk.Label(ventana, text="Binario:\n", bg="#2C3E50", fg="white", font=("Arial", 12))
etiqueta_resultado.pack(pady=5)

boton_archivo = tk.Button(ventana, text="Procesar Archivo de Instrucciones", command=procesar_archivo, bg="#F39C12", fg="white", font=("Arial", 12), padx=10, pady=5)
boton_archivo.pack(pady=5)

frame_guardado = tk.Frame(ventana, bg="#2C3E50")
frame_guardado.pack(pady=5)

entrada_archivo = tk.Entry(frame_guardado, textvariable=ruta_archivo, width=40, font=("Arial", 12), bg="#ECF0F1", fg="#2C3E50")
entrada_archivo.pack(side=tk.LEFT, padx=5)

boton_seleccionar = tk.Button(frame_guardado, text="Seleccionar Archivo de Salida", command=seleccionar_archivo_salida, bg="#E74C3C", fg="white", font=("Arial", 12), padx=10, pady=5)
boton_seleccionar.pack(side=tk.RIGHT)

# Ejecutar la aplicación
ventana.mainloop()
