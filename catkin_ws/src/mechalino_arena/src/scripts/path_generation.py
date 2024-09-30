import numpy as np

def generate_rectangle_path(x1, y1, x2, y2, m, n):
    # Berechne die Länge und Breite des Rechtecks
    length = abs(x2 - x1)
    width = abs(y2 - y1)
    
    # Unterteile die Länge in n Stücke und die Breite in m Stücke
    x_points = np.linspace(x1, x2, n + 1)
    y_points = np.linspace(y1, y2, m + 1)
    
    path = []
    
    # Generiere die Punkte des Pfades
    for i in range(len(y_points)):
        if i % 2 == 0:
            # Wenn gerade Zeile, Punkte von links nach rechts
            for x in x_points:
                path.append((x, y_points[i]))
        else:
            # Wenn ungerade Zeile, Punkte von rechts nach links
            for x in reversed(x_points):
                path.append((x, y_points[i]))
    
    return path