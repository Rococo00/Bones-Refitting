# Allineamento di frammenti 3D

Un programma Python per allineare due frammenti di un oggetto 3D partendo da alcuni punti scelti manualmente sulle loro superfici.

L'idea è semplice: indichi quali punti dei due frammenti devono corrispondere e il programma usa queste indicazioni per avvicinarli. Poi cerca di migliorare l'allineamento nelle zone attorno ai punti selezionati, usando l'algoritmo ICP (*Iterative Closest Point*).

Il primo frammento, mostrato in blu, rimane fermo. Il secondo, in rosso, viene ruotato e traslato per allinearsi al primo.

## Cosa serve

- Python e `pip`. La versione indicata per l'ambiente di sviluppo è **Python 3.12**, da confermare con `python --version`.
- Tkinter, usato per le finestre di selezione dei file.
- Un ambiente desktop in cui aprire le finestre 3D di Open3D.
- Due mesh dei frammenti da allineare, nella stessa scala e unità di misura.

La finestra di selezione propone i formati `.obj`, `.ply`, `.stl` e `.off`. I file devono contenere mesh valide: il programma usa i loro vertici e le normali per eseguire l'allineamento.

## Installazione

Scarica il repository e apri un terminale nella cartella del progetto.

È consigliabile creare un ambiente virtuale:

```bash
python -m venv .venv
```

Attivalo su Windows, dal Prompt dei comandi:

```bat
.venv\Scripts\activate.bat
```

Oppure su Linux e macOS:

```bash
source .venv/bin/activate
```

Le sole dipendenze esterne importate direttamente dallo script sono **Open3D** e **NumPy**. Per un `requirements.txt` essenziale, usa:

```txt
open3d==0.19.0
numpy==2.4.6
```

Le versioni riportate provengono dal file delle dipendenze dell'ambiente di origine. Le eventuali dipendenze indirette vengono installate automaticamente da `pip`.

Installa le dipendenze:

```bash
python -m pip install -r requirements.txt
```

Python non va inserito in `requirements.txt`: deve essere già installato sul computer. Anche `copy`, che fa parte della libreria standard, non richiede un'installazione separata.

Tkinter non viene installato da questo comando: deve essere disponibile nell'installazione di Python. Puoi verificarlo con `python -m tkinter`: se è presente, si apre una piccola finestra.

## Come si usa

Avvia lo script:

```bash
python "main.py"
```

Se nel repository lo rinomini in `main.py`, usa invece `python main.py`.

1. **Scegli il primo frammento.** È il riferimento, visualizzato in blu, e resterà fermo.
2. **Scegli il secondo frammento.** Sarà visualizzato in rosso e verrà spostato.
3. **Seleziona almeno quattro punti sul primo frammento**, usando **Shift + clic sinistro**. Premi **Q** per terminare la selezione.
4. **Seleziona i punti corrispondenti sul secondo frammento**, nello stesso ordine e nello stesso numero. Premi nuovamente **Q**.
5. Il programma calcola l'allineamento e apre una finestra con le due mesh nella posizione finale.

L'ordine è importante: il primo punto scelto sul frammento blu viene associato al primo punto scelto sul rosso, il secondo al secondo, e così via.

Per ottenere un risultato utile, scegli punti riconoscibili sulla zona di frattura o di contatto, distribuiti sulla superficie anziché tutti concentrati nello stesso punto o lungo una sola linea.

## Come funziona

L'allineamento avviene in quattro passaggi:

1. **Caricamento delle mesh.** Il programma calcola le normali e usa i vertici delle mesh per costruire due nuvole di punti.
2. **Allineamento iniziale.** Dai punti scelti manualmente ricava una rotazione e una traslazione del secondo frammento. La scala degli oggetti non viene modificata.
3. **Estrazione delle patch.** Per ciascun frammento seleziona i punti che si trovano entro un certo raggio dai punti indicati. Queste piccole regioni vengono chiamate *patch*. Non c'è un riconoscimento automatico della frattura: le regioni dipendono dalla selezione manuale.
4. **Raffinamento con ICP.** Applica ICP point-to-plane alle patch per migliorare l'allineamento iniziale, con un massimo di 100 iterazioni. La trasformazione complessiva viene poi applicata alla mesh del secondo frammento.

## Parametri modificabili

All'inizio dello script trovi due parametri:

```python
RAGGIO_PATCH = 0.06
SOGLIA_ICP = 0.004
```

Entrambi sono proporzionali alla dimensione dei modelli. Il programma calcola la diagonale del bounding box di ogni frammento, cioè del parallelepipedo allineato agli assi che lo contiene, e usa la media delle due diagonali come riferimento.

| Parametro | A cosa serve | Valore predefinito |
| --- | --- | --- |
| `RAGGIO_PATCH` | Determina quanto sono ampie le regioni attorno ai punti selezionati. | 6% della dimensione di riferimento |
| `SOGLIA_ICP` | Stabilisce la distanza massima entro cui ICP considera due punti come possibili corrispondenze. | 0,4% della dimensione di riferimento |

Se le patch contengono pochi punti, puoi aumentare `RAGGIO_PATCH`. Un raggio troppo grande, però, può includere superfici che non appartengono alla zona da allineare.

Una soglia ICP troppo piccola può escludere corrispondenze utili; una troppo grande può ammetterne di sbagliate. Prima di modificarla, controlla di aver selezionato correttamente i punti.

## Risultati

Oltre alla visualizzazione finale, il terminale mostra il numero di punti delle patch e due indicatori di ICP:

- **Fitness:** indica la quota di punti della patch sorgente che trovano una corrispondenza entro la soglia impostata.
- **RMSE:** misura l'errore quadratico medio, sotto radice, delle distanze tra le corrispondenze accettate, nelle unità dei modelli.

Questi valori aiutano a valutare l'allineamento, ma vanno letti insieme e confrontati con il risultato visivo. Un errore basso, da solo, non garantisce che i frammenti siano stati ricomposti correttamente.

**La versione attuale visualizza il risultato senza esportarlo su file.** I file originali non vengono modificati.

## Problemi comuni

| Problema | Cosa controllare |
| --- | --- |
| Il programma richiede almeno quattro punti. | Seleziona almeno quattro punti per frammento, nello stesso numero e ordine. |
| Compare il messaggio “patch troppo piccole”. | Almeno una patch ha meno di 10 punti: aumenta `RAGGIO_PATCH` o scegli una zona con più vertici. |
| Una mesh risulta vuota o non caricata. | Controlla il percorso e verifica che il file contenga una mesh valida. |
| I frammenti si allineano male. | Controlla le corrispondenze manuali, la scala dei modelli e la scelta delle regioni di contatto. |
| Non si aprono le finestre. | Verifica la disponibilità di Tkinter e di un ambiente grafico adatto alla visualizzazione 3D. |

Il programma richiede una selezione manuale iniziale: ICP raffina quella posizione e può convergere a un allineamento sbagliato se i punti di partenza non sono adatti. Inoltre, non esegue fusione delle mesh, ricostruzione delle parti mancanti o controllo delle collisioni tra i frammenti.
