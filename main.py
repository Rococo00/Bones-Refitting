"""
Allineamento di due frammenti 3D.
Pipeline:
    1) selezione manuale dei punti corrispondenti
    2) rotazione + traslazione iniziale (coarse)
    3) estrazione delle patch di frattura
    4) allineamento fine con ICP
"""


#importazione delle librerie
import open3d as o3d
import numpy as np
import copy
import tkinter as tk
from tkinter import filedialog

#frazione della dimensione dell'osso (6% e 0.4%)
RAGGIO_PATCH = 0.06     #dimensione sfera ritagliata attorno a ogni punto
SOGLIA_ICP = 0.004      #distanza massima perché 2 punti vengano considerati in ICP come 'corrispondenti'


#funzione per scegliere i file degli oggetti 3D
def scegli_file(titolo):
    root = tk.Tk()
    root.withdraw()          #nasconde la finestra principale vuota
    root.attributes("-topmost", True)

    path = filedialog.askopenfilename(
        title=titolo,
        filetypes=[
            ("Mesh", "*.obj *.ply *.stl *.off"),
            ("Tutti i file", "*.*"),
        ],
    )
    root.destroy()
    return path

#da file a nuvola di punti con normali
def carica(filename, colore):
    mesh = o3d.io.read_triangle_mesh(filename) #legge vertici + triangoli
    if mesh.is_empty():
        raise RuntimeError(f"Mesh '{filename}' vuota o non caricata.")
    mesh.compute_vertex_normals() #calcola la normale di ogni vertice

    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.normals = mesh.vertex_normals
    pcd.paint_uniform_color(colore)
    return pcd, mesh

#serve a misurare la 'grandezza' dell'osso
def scala(pcd):
    bbox = pcd.get_axis_aligned_bounding_box() #calcolo la bounding box
    return np.linalg.norm(bbox.get_max_bound() - bbox.get_min_bound()) #e poi ci calcolo la diagonale

#funzione con cui utente seleziona i punti
def seleziona_punti(pcd, titolo):
    print(f"\n--- {titolo} ---") #output nel terminale
    print("SHIFT + clic sinistro per selezionare un punto (almeno 4).")
    print("Stesso ordine sui due frammenti. Q per confermare.")

    vis = o3d.visualization.VisualizerWithEditing() #visualizzatore con cui selezionare i punti
    vis.create_window(window_name=titolo)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    return vis.get_picked_points() #restituisce gli indici dei punti selezionati dall'utente

#funzione per 'ritagliare' la zona di frattura
def patch_attorno_a(pcd, centri, raggio):
    """Patch = unione di sfere attorno ai punti selezionati."""
    tree = o3d.geometry.KDTreeFlann(pcd) #struttura dati per cercare velocemente i punti vicini
    indici = set() #per eliminare eventuali duplicati
    for c in centri: #per ogni punto selezionato dall'utente
        _, idx, _ = tree.search_radius_vector_3d(c, raggio) #trovo i punti entro il raggio
        indici.update(idx)
    return pcd.select_by_index(list(indici)) #restituisce una nuvola di punti

#mostra solo le geometrie passate
def mostra(titolo, geometrie):
    o3d.visualization.draw_geometries(
        geometrie, window_name=titolo, width=1200, height=800
    )


def main():
    #selezionare i 2 frammenti
    file1 = scegli_file("Frammento 1 (BLU) - resta fermo") 
    file2 = scegli_file("Frammento 2 (ROSSO) - viene mosso")

    if not file1 or not file2:
        print("Nessun file selezionato.")
        return
    
    #colora i 2 frammenti
    pcd1, mesh1 = carica(file1, [0.0, 0.3, 1.0])
    pcd2, mesh2 = carica(file2, [1.0, 0.0, 0.0])

    s = (scala(pcd1) + scala(pcd2)) / 2.0
    raggio = s * RAGGIO_PATCH
    soglia = s * SOGLIA_ICP

    #mostra i frammenti appena caricati
    mostra("prima di iniziare", [pcd1, pcd2])

    #Fase 1: Selezione dei punti (utente)
    id1 = seleziona_punti(pcd1, "Frammento 1 (BLU) - seleziona i punti")
    id2 = seleziona_punti(pcd2, "Frammento 2 (ROSSO) - stessi punti, stesso ordine")

    if len(id1) < 4 or len(id1) != len(id2):
        print("ERRORE: servono almeno 4 punti, in numero uguale sui due frammenti.")
        return

    punti1 = np.asarray(pcd1.points)[id1]
    punti2 = np.asarray(pcd2.points)[id2]




    #Fase 2: Rotazione e traslazione iniziale

    #associa ad ogni punto del frammento rosso a quelli del blu, cliccati in ordine
    corr = o3d.utility.Vector2iVector(np.array([[i, i] for i in range(len(id1))]))
    T_iniziale = o3d.pipelines.registration.TransformationEstimationPointToPoint(
    ).compute_transformation(
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(punti2)),
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(punti1)),
        corr,
    )

    pcd2_coarse = copy.deepcopy(pcd2).transform(T_iniziale)
    punti2_coarse = (T_iniziale[:3, :3] @ punti2.T).T + T_iniziale[:3, 3]

    mostra("dopo roto traslazione ", [pcd1, pcd2_coarse])
    #Fase Patch di frattura
    patch1 = patch_attorno_a(pcd1, punti1, raggio) #zona di frattura sul blu
    patch2 = patch_attorno_a(pcd2_coarse, punti2_coarse, raggio) #zona frattura sul rosso
    print(f"\nPatch: {len(patch1.points)} e {len(patch2.points)} punti.")

    if len(patch1.points) < 10 or len(patch2.points) < 10: #se patch troppe piccole.....
        print("ERRORE: patch troppo piccole, aumenta RAGGIO_PATCH.")
        return

    #Fase 4: ICP point-to-plane sulle patch)
    icp = o3d.pipelines.registration.registration_icp(
        patch2, #rosso
        patch1, #blu
        soglia, #distanza massima per corrispondenza
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
    )
    #Fitness: punti della patch rossa che hanno un corrispondente entro la soglia
    #RMSE: Media delle distanze al quadrato
    print(f"ICP -> fitness {icp.fitness:.4f}, RMSE {icp.inlier_rmse:.6f}")

    T_finale = icp.transformation @ T_iniziale

    #RISULTATO
    mesh2.transform(T_finale)
    mesh1.paint_uniform_color([0.2, 0.5, 0.8])
    mesh2.paint_uniform_color([0.8, 0.2, 0.2])
    mesh1.compute_vertex_normals()
    mesh2.compute_vertex_normals()

    mostra("Risultato finale", [mesh1, mesh2])


if __name__ == "__main__":
    main()
