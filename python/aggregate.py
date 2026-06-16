"""
aggregate.py — Aggregazione risultati sperimentali Swarm Firefighter.

Legge i file summary.txt prodotti da monitor.py per un insieme di run
e calcola media ± deviazione standard delle metriche chiave.

════════════════════════════════════════════════════════════════════════
STRUTTURA ATTESA DEI FILE
════════════════════════════════════════════════════════════════════════

I summary.txt devono essere nella struttura prodotta da monitor.py:

  plots/
    run_20250601_100000/swarm/summary.txt   ← N=5, run 1
    run_20250601_101500/swarm/summary.txt   ← N=5, run 2
    ...
    run_20250601_120000/swarm/summary.txt   ← N=7, run 1
    ...

Formato atteso di ogni summary.txt (prodotto da monitor._save_summary):

  Label               : n8_damp_off_rep1
  N_DRONES            : 5
  Durata sessione     : 900.3 s  (15.0 min)
  Incendi spenti      : 12
  Dist. min osservata : 4.83 m  (D_SAFE = 4.0 m)
  Tempo sotto D_SAFE  : 0.12 %  (2 incursioni: 2 transito, 0 standoff)
  Media fuochi attivi : 2.41

Le righe "Label" e "Tempo sotto D_SAFE" sono opzionali: i summary nel
vecchio formato vengono letti comunque (label = "-", metriche = n/d).

RAGGRUPPAMENTO
  Le run sono raggruppate per (N_DRONES, condizione), dove la condizione
  è la label normalizzata: gli eventuali suffissi di ripetizione
  ("rep1", "_rep2", ...) vengono rimossi, così run etichettate
  "damp_off_rep1" e "damp_off_rep2" finiscono nello stesso gruppo
  "damp_off". Questo tiene separate le condizioni dell'ablazione
  (damp on/off) anche a parità di N.

════════════════════════════════════════════════════════════════════════
USO
════════════════════════════════════════════════════════════════════════

  # Aggrega automaticamente tutte le run trovate sotto plots/
  python aggregate.py

  # Specifica una cartella radice diversa
  python aggregate.py --plots-dir /path/to/plots

  # Esclude i primi N secondi di ogni run dall'analisi
  # (utile se si vuole scartare la fase di warm-up)
  python aggregate.py --warmup 30

  # Stampa anche i valori delle singole run (debug)
  python aggregate.py --verbose

════════════════════════════════════════════════════════════════════════
OUTPUT
════════════════════════════════════════════════════════════════════════

Stampa su stdout una tabella testuale e salva aggregate_results.txt
nella cartella --plots-dir.

Esempio output:

  ┌─────────┬──────────────────────┬───────────────────────┬────────────────────┐
  │ N droni │ Incendi spenti       │ Fuochi medi attivi    │ Dist. min ass. [m] │
  ├─────────┼──────────────────────┼───────────────────────┼────────────────────┤
  │    5    │    8.4 ± 1.5         │    3.21 ± 0.44        │       4.31         │
  │    7    │   11.2 ± 1.1         │    2.87 ± 0.31        │       4.12         │
  │    9    │   13.6 ± 0.9         │    2.43 ± 0.28        │       3.97         │
  └─────────┴──────────────────────┴───────────────────────┴────────────────────┘

  Nota: Dist. min ass. = minimo assoluto tra tutte le run (worst-case).
"""

import os
import re
import sys
import math
import argparse
from collections import defaultdict


# ─────────────────────────────────────────────────────────────────────────────
# Parsing argomenti
# ─────────────────────────────────────────────────────────────────────────────

def _parse_args():
    parser = argparse.ArgumentParser(
        description="Aggrega i summary.txt delle sessioni Swarm Firefighter."
    )
    parser.add_argument(
        "--plots-dir", default="plots",
        help="Cartella radice contenente le sottocartelle run_* (default: plots/)"
    )
    parser.add_argument(
        "--warmup", type=float, default=0.0,
        help="Secondi iniziali da ignorare (default: 0 — nessun warm-up escluso)"
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="Stampa i valori di ogni singola run"
    )
    return parser.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# Lettura di un singolo summary.txt
# ─────────────────────────────────────────────────────────────────────────────

def _parse_summary(path: str) -> dict | None:
    """
    Legge un summary.txt e restituisce un dict con le metriche numeriche.

    Campi restituiti:
      n_drones          : int
      label             : str     — etichetta della run ("-" se assente)
      duration_s        : float   — durata in secondi
      fires_extinguished: int
      min_dist          : float   — distanza minima inter-drone [m]
      avg_active_fires  : float   — media fuochi attivi
      pct_below         : float   — % tempo sotto D_SAFE (NaN se assente)
      incursions        : float   — numero incursioni totali (NaN se assente)
      inc_transit       : float   — incursioni in transito (NaN se assente)
      inc_standoff      : float   — incursioni allo standoff (NaN se assente)

    Restituisce None se il file non è leggibile o mancano campi obbligatori.
    """
    fields = {
        "label"       : "-",
        "pct_below"   : float("nan"),
        "incursions"  : float("nan"),
        "inc_transit" : float("nan"),
        "inc_standoff": float("nan"),
    }
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                # Formato: "Campo   : valore"
                if ":" not in line:
                    continue
                key, _, rest = line.partition(":")
                key  = key.strip()
                rest = rest.strip()

                if key == "Label":
                    fields["label"] = rest if rest else "-"

                elif key == "N_DRONES":
                    fields["n_drones"] = int(rest)

                elif key == "Durata sessione":
                    # "900.3 s  (15.0 min)"
                    m = re.search(r"([\d.]+)\s*s", rest)
                    if m:
                        fields["duration_s"] = float(m.group(1))

                elif key == "Incendi spenti":
                    fields["fires_extinguished"] = int(rest)

                elif key == "Dist. min osservata":
                    # "4.83 m  (D_SAFE = 4.0 m)"
                    m = re.search(r"([\d.]+)\s*m", rest)
                    if m:
                        fields["min_dist"] = float(m.group(1))

                elif key == "Tempo sotto D_SAFE":
                    # Nuovo formato:
                    #   "0.12 %  (2 incursioni: 2 transito, 0 standoff)"
                    # Vecchio formato (senza classificazione):
                    #   "0.12 %  (2 incursioni)"
                    m = re.search(r"([\d.]+|nan)\s*%", rest)
                    if m:
                        try:
                            fields["pct_below"] = float(m.group(1))
                        except ValueError:
                            pass
                    m = re.search(r"\((\d+)\s+incursioni", rest)
                    if m:
                        fields["incursions"] = float(m.group(1))
                    m = re.search(r"(\d+)\s+transito,\s*(\d+)\s+standoff", rest)
                    if m:
                        fields["inc_transit"]  = float(m.group(1))
                        fields["inc_standoff"] = float(m.group(2))

                elif key == "Media fuochi attivi":
                    # "nan" se non disponibile
                    try:
                        fields["avg_active_fires"] = float(rest)
                    except ValueError:
                        fields["avg_active_fires"] = float("nan")

    except OSError as e:
        print(f"  [WARN] impossibile leggere {path}: {e}", file=sys.stderr)
        return None

    required = {"n_drones", "fires_extinguished", "min_dist"}
    missing  = required - fields.keys()
    if missing:
        print(f"  [WARN] {path}: campi mancanti {missing}", file=sys.stderr)
        return None

    return fields


# ─────────────────────────────────────────────────────────────────────────────
# Raccolta di tutti i summary.txt sotto plots/
# ─────────────────────────────────────────────────────────────────────────────

def _normalize_label(label: str) -> str:
    """
    Normalizza la label in una "condizione" rimuovendo i suffissi di
    ripetizione: "damp_off_rep1" e "damp_off_rep2" → "damp_off".
    Run senza label ("-") restano nella condizione "-".
    """
    if not label or label == "-":
        return "-"
    cond = re.sub(r"[_-]?rep\d+", "", label, flags=re.IGNORECASE)
    cond = cond.strip("_- ")
    return cond if cond else "-"


def _collect_summaries(plots_dir: str, verbose: bool) -> dict:
    """
    Scansiona ricorsivamente plots_dir cercando file summary.txt.
    Raggruppa i risultati per (n_drones, condizione), dove la condizione
    è la label normalizzata (vedi _normalize_label). Questo tiene separate
    le condizioni dell'ablazione (es. damp on/off) a parità di N.

    Restituisce:
      { (n_drones, condizione): [ {metriche run 1}, ... ], ... }
    """
    if not os.path.isdir(plots_dir):
        print(f"Errore: cartella '{plots_dir}' non trovata.", file=sys.stderr)
        sys.exit(1)

    groups = defaultdict(list)
    found  = 0

    for root, dirs, files in os.walk(plots_dir):
        # Cerca solo nella sottocartella swarm/ di ogni run
        if os.path.basename(root) != "swarm":
            continue
        summary_path = os.path.join(root, "summary.txt")
        if not os.path.isfile(summary_path):
            continue

        data = _parse_summary(summary_path)
        if data is None:
            continue

        n    = data["n_drones"]
        cond = _normalize_label(data.get("label", "-"))
        data["condition"] = cond
        groups[(n, cond)].append(data)
        found += 1

        if verbose:
            print(f"  [{summary_path}]")
            print(f"    N={n}  cond='{cond}'  spenti={data['fires_extinguished']}"
                  f"  min_dist={data['min_dist']:.2f}m"
                  f"  avg_fires={data.get('avg_active_fires', float('nan')):.2f}"
                  f"  pct<D_SAFE={data['pct_below']:.2f}%")

    print(f"\nTrovati {found} summary.txt in '{plots_dir}'.")
    return dict(groups)


# ─────────────────────────────────────────────────────────────────────────────
# Statistica descrittiva
# ─────────────────────────────────────────────────────────────────────────────

def _mean(values: list) -> float:
    return sum(values) / len(values) if values else float("nan")

def _std(values: list) -> float:
    """Deviazione standard campionaria (ddof=1)."""
    n = len(values)
    if n < 2:
        return float("nan")
    m = _mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (n - 1))

def _mean_std(values: list) -> tuple:
    """Restituisce (media, std) escludendo NaN."""
    clean = [v for v in values if not math.isnan(v)]
    return _mean(clean), _std(clean)


# ─────────────────────────────────────────────────────────────────────────────
# Aggregazione per configurazione N
# ─────────────────────────────────────────────────────────────────────────────

def _aggregate(groups: dict) -> list:
    """
    Per ogni configurazione (N, condizione) calcola le statistiche aggregate.

    Restituisce una lista di dict ordinata per (N, condizione):
      {
        n            : int
        condition    : str     — condizione (label normalizzata)
        n_runs       : int
        ext_mean     : float   — media incendi spenti
        ext_std      : float   — std incendi spenti
        fires_mean   : float   — media fuochi attivi
        fires_std    : float   — std fuochi attivi
        min_dist_wc  : float   — minimo assoluto dist. inter-drone (worst-case)
        pct_mean     : float   — media % tempo sotto D_SAFE
        pct_std      : float   — std % tempo sotto D_SAFE
        inc_mean     : float   — media incursioni per run
        inc_std      : float   — std incursioni per run
        inc_transit  : float   — totale incursioni in transito (somma sulle run)
        inc_standoff : float   — totale incursioni allo standoff (somma sulle run)
      }
    """
    results = []
    for (n, cond) in sorted(groups.keys()):
        runs = groups[(n, cond)]

        ext_vals   = [r["fires_extinguished"]                   for r in runs]
        fires_vals = [r.get("avg_active_fires", float("nan"))   for r in runs]
        dist_vals  = [r["min_dist"]                             for r in runs]
        pct_vals   = [r["pct_below"]                            for r in runs]
        inc_vals   = [r["incursions"]                           for r in runs]
        tra_vals   = [r["inc_transit"]  for r in runs if not math.isnan(r["inc_transit"])]
        sta_vals   = [r["inc_standoff"] for r in runs if not math.isnan(r["inc_standoff"])]

        ext_mean,   ext_std   = _mean_std(ext_vals)
        fires_mean, fires_std = _mean_std(fires_vals)
        pct_mean,   pct_std   = _mean_std(pct_vals)
        inc_mean,   inc_std   = _mean_std(inc_vals)
        min_dist_wc = min(dist_vals)   # worst-case: minimo tra le run

        results.append({
            "n"           : n,
            "condition"   : cond,
            "n_runs"      : len(runs),
            "ext_mean"    : ext_mean,
            "ext_std"     : ext_std,
            "fires_mean"  : fires_mean,
            "fires_std"   : fires_std,
            "min_dist_wc" : min_dist_wc,
            "pct_mean"    : pct_mean,
            "pct_std"     : pct_std,
            "inc_mean"    : inc_mean,
            "inc_std"     : inc_std,
            "inc_transit" : sum(tra_vals) if tra_vals else float("nan"),
            "inc_standoff": sum(sta_vals) if sta_vals else float("nan"),
        })

    return results


# ─────────────────────────────────────────────────────────────────────────────
# Formattazione output
# ─────────────────────────────────────────────────────────────────────────────

def _fmt_mean_std(mean: float, std: float, decimals: int = 1) -> str:
    """Formatta 'mean ± std' con il numero di decimali specificato."""
    if math.isnan(mean):
        return "n/d"
    if math.isnan(std):
        return f"{mean:.{decimals}f}"
    return f"{mean:.{decimals}f} ± {std:.{decimals}f}"

def _fmt_dist(val: float) -> str:
    return f"{val:.2f}" if not math.isnan(val) else "n/d"


def _render_table(results: list) -> str:
    """
    Produce la tabella testuale dei risultati aggregati.
    Una riga per ogni coppia (N, condizione).
    """
    headers = [
        "N", "Condizione", "Run",
        "Incendi spenti", "Fuochi medi attivi",
        "Dist. min ass. [m]", "% t < D_SAFE", "Incursioni (tra/sta)",
    ]

    def _fmt_inc(r) -> str:
        if math.isnan(r["inc_mean"]):
            return "n/d"
        base = _fmt_mean_std(r["inc_mean"], r["inc_std"], decimals=1)
        if not math.isnan(r["inc_transit"]):
            base += f"  ({int(r['inc_transit'])}/{int(r['inc_standoff'])})"
        return base

    rows = []
    for r in results:
        rows.append((
            str(r["n"]),
            r["condition"],
            str(r["n_runs"]),
            _fmt_mean_std(r["ext_mean"],   r["ext_std"],   decimals=1),
            _fmt_mean_std(r["fires_mean"], r["fires_std"], decimals=2),
            _fmt_dist(r["min_dist_wc"]),
            _fmt_mean_std(r["pct_mean"],   r["pct_std"],   decimals=2),
            _fmt_inc(r),
        ))

    widths = [max(len(h), max((len(row[i]) for row in rows), default=0))
              for i, h in enumerate(headers)]

    sep = "+-" + "-+-".join("-" * w for w in widths) + "-+"
    hdr = "| " + " | ".join(h.ljust(w) for h, w in zip(headers, widths)) + " |"

    lines = [sep, hdr, sep]
    for row in rows:
        lines.append(
            "| " + " | ".join(c.ljust(w) for c, w in zip(row, widths)) + " |"
        )
    lines.append(sep)
    lines.append("")
    lines.append("Note: 'Dist. min ass.' = minimo assoluto tra tutte le run (worst-case).")
    lines.append("      'Incursioni'     = episodi sotto D_SAFE per run (media ± std);")
    lines.append("                         tra parentesi i totali (transito/standoff).")

    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────────────
# Salvataggio risultati
# ─────────────────────────────────────────────────────────────────────────────

def _save_results(table: str, results: list, plots_dir: str):
    """
    Salva aggregate_results.txt con la tabella e i valori raw per ogni N.
    """
    out_path = os.path.join(plots_dir, "aggregate_results.txt")
    lines = [
        "═" * 70,
        "RISULTATI AGGREGATI — Swarm Firefighter",
        "═" * 70,
        "",
        table,
        "",
        "─" * 70,
        "VALORI PER CONFIGURAZIONE (per costruire la tabella LaTeX)",
        "─" * 70,
    ]
    for r in results:
        lines += [
            "",
            f"N = {r['n']}, condizione = '{r['condition']}'  ({r['n_runs']} run)",
            f"  Incendi spenti      : {r['ext_mean']:.2f} ± {r['ext_std']:.2f}",
            f"  Fuochi medi attivi  : {r['fires_mean']:.3f} ± {r['fires_std']:.3f}",
            f"  Dist. min ass.      : {r['min_dist_wc']:.2f} m",
            f"  % tempo < D_SAFE    : {r['pct_mean']:.2f} ± {r['pct_std']:.2f}",
            f"  Incursioni per run  : {r['inc_mean']:.1f} ± {r['inc_std']:.1f}"
            + ("" if math.isnan(r["inc_transit"]) else
               f"  (totali: {int(r['inc_transit'])} transito, "
               f"{int(r['inc_standoff'])} standoff)"),
        ]

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"\nRisultati salvati in: {out_path}")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    args = _parse_args()

    if args.warmup > 0:
        print(f"[INFO] --warmup {args.warmup}s specificato.")
        print( "       Il filtro warm-up agisce sulla durata della sessione")
        print( "       ma non sui contatori di summary.txt (già aggregati).")
        print( "       Per escludere il warm-up in modo preciso, rigira le")
        print( "       sessioni con monitor.py e tronca manualmente la finestra.")

    if args.verbose:
        print("\nDettaglio run trovate:")

    groups  = _collect_summaries(args.plots_dir, args.verbose)

    if not groups:
        print("Nessun summary.txt trovato. Verifica il percorso --plots-dir.")
        sys.exit(1)

    results = _aggregate(groups)

    print()
    table = _render_table(results)
    print(table)

    _save_results(table, results, args.plots_dir)

    # Avviso se qualche configurazione ha meno di 5 run
    for r in results:
        tag = f"N={r['n']}, cond='{r['condition']}'"
        if r["n_runs"] < 5:
            print(f"\n[WARN] {tag}: solo {r['n_runs']} run trovate (attese 5).")
        elif r["n_runs"] > 5:
            print(f"\n[INFO] {tag}: {r['n_runs']} run trovate (più di 5 — tutte incluse).")


if __name__ == "__main__":
    main()