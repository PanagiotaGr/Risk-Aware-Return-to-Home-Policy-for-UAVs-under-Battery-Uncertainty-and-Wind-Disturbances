# Risk-Aware Return-to-Home Policy for UAVs

## Ελληνική αναλυτική τεκμηρίωση

Το project μελετά πολιτικές Return-to-Home για UAVs υπό αβεβαιότητα μπαταρίας, γήρανση μπαταρίας και δυναμικές διαταραχές ανέμου. Η βασική ιδέα είναι ότι ένα UAV δεν πρέπει να αποφασίζει την επιστροφή του μόνο με ένα σταθερό όριο SoC, αλλά με βάση την πιθανότητα ασφαλούς επιστροφής.

Η προσέγγιση χρησιμοποιεί Monte Carlo sampling για να εκτιμήσει αν η διαθέσιμη ενέργεια επαρκεί για επιστροφή στο home point, λαμβάνοντας υπόψη αβέβαιες μετρήσεις, wind disturbance και battery health.

## Ερευνητικό ερώτημα

Το βασικό ερώτημα είναι:

Μπορούν οι probabilistic και health-aware RTH πολιτικές να βελτιώσουν την ασφάλεια επιστροφής UAVs σε σχέση με deterministic threshold-based πολιτικές;

## Πολιτικές που συγκρίνονται

| Πολιτική | Περιγραφή |
| --- | --- |
| deterministic_threshold | ενεργοποιεί RTH όταν το estimated SoC πέσει κάτω από σταθερό όριο |
| risk_aware_mc | εκτιμά πιθανότητα ασφαλούς επιστροφής με Monte Carlo sampling |
| adaptive_risk_mc | προσαρμόζει το risk threshold με βάση άνεμο, απόσταση, αβεβαιότητα και battery health |
| health_aware_risk_mc | λαμβάνει ρητά υπόψη την υποβάθμιση της μπαταρίας |

## Κανόνας απόφασης

Ο planner εκτιμά την πιθανότητα ασφαλούς επιστροφής. Αν αυτή η πιθανότητα πέσει κάτω από το risk threshold, τότε ενεργοποιείται Return-to-Home.

Στη health-aware πολιτική, το διαθέσιμο SoC προσαρμόζεται με βάση τον παράγοντα battery health. Έτσι το σύστημα δεν υποθέτει ότι μία γερασμένη μπαταρία έχει την ίδια πραγματική απόδοση με μία καινούρια.

## Monte Carlo risk estimation

Η Monte Carlo διαδικασία εκτελεί πολλά υποθετικά δείγματα της κατάστασης:

- πιθανό πραγματικό SoC,
- πιθανή ένταση ανέμου,
- πιθανή επιπλέον ενεργειακή κατανάλωση,
- απαιτούμενη ενέργεια επιστροφής.

Για κάθε δείγμα ελέγχεται αν η διαθέσιμη ενέργεια επαρκεί. Το ποσοστό των επιτυχημένων δειγμάτων αποτελεί την πιθανότητα ασφαλούς επιστροφής.

## Σενάρια πειραμάτων

Το project εξετάζει:

- διαφορετικές κατευθύνσεις ανέμου,
- διαφορετικές εντάσεις ανέμου,
- battery measurement noise,
- optimistic και pessimistic SoC bias,
- fault injection με extra drain,
- battery aging,
- dynamic wind gusts,
- ablation studies σε thresholds και Monte Carlo samples.

## Μετρικές αξιολόγησης

| Μετρική | Ερμηνεία |
| --- | --- |
| RTH trigger rate | πόσο συχνά ενεργοποιείται επιστροφή |
| safe return rate | πόσο συχνά η επιστροφή είναι ασφαλής |
| failure rate | ποσοστό αποτυχίας |
| mission completion rate | ποσοστό ολοκλήρωσης αποστολής |
| mean energy margin | μέσο ενεργειακό περιθώριο |
| negative margin rate | πόσο συχνά η ενέργεια δεν επαρκεί |

## Κύρια αποτελέσματα

| Policy | RTH Trigger | Safe Return | Failure | Mission Completion |
| --- | ---: | ---: | ---: | ---: |
| deterministic_threshold | 0.15% | 0.15% | 20.54% | 99.47% |
| risk_aware_mc | 40.33% | 19.94% | 20.83% | 78.79% |
| adaptive_risk_mc | 52.23% | 31.00% | 21.27% | 73.31% |
| health_aware_risk_mc | 39.90% | 19.90% | 20.75% | 79.04% |

Η deterministic πολιτική έχει υψηλό mission completion επειδή σπάνια ενεργοποιεί RTH. Όμως η συμπεριφορά safe return είναι πολύ χαμηλή. Οι Monte Carlo πολιτικές είναι πιο συντηρητικές και αυξάνουν σημαντικά το safe return.

## Battery aging

Η υποβάθμιση της μπαταρίας επηρεάζει έντονα την ασφάλεια. Όσο μειώνεται το battery health, αυξάνεται η πιθανότητα αρνητικού energy margin. Η health-aware πολιτική είναι πιο κατάλληλη για degraded batteries, επειδή λαμβάνει υπόψη την πραγματική μείωση διαθέσιμης ενέργειας.

## Dynamic wind gusts

Οι δυναμικές ριπές ανέμου αυξάνουν την αβεβαιότητα της επιστροφής. Σε low και medium gust scenarios οι Monte Carlo πολιτικές βελτιώνουν σημαντικά το safe return. Σε high και extreme gust scenarios, η επιστροφή γίνεται δύσκολη για όλες τις πολιτικές, αλλά οι risk-aware πολιτικές παραμένουν πιο πληροφορημένες από το deterministic baseline.

## Ablation studies

Τα ablation studies εξετάζουν:

- deterministic SoC threshold,
- risk threshold tau,
- αριθμό Monte Carlo samples.

Το συμπέρασμα είναι ότι περισσότερα samples αυξάνουν το runtime σχεδόν γραμμικά, ενώ η ποιότητα απόφασης σταθεροποιείται σχετικά γρήγορα. Πρακτικά, 50 έως 100 samples μπορούν συχνά να δώσουν αρκετά σταθερή απόφαση.

## Εκτέλεση

```bash
git clone https://github.com/PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances.git
cd Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances
python3 -m venv .venv
source .venv/bin/activate
pip install "numpy<2" matplotlib
python3 experiments/run_monte_carlo_experiments.py --trials 200 --mc-samples 500
python3 experiments/run_ablation_studies.py --trials 200
python3 experiments/run_battery_aging_study.py --trials 200
python3 experiments/run_wind_gust_study.py --trials 200
```

## Τι κάναμε αναλυτικά

Στο repository προστέθηκε αναλυτική ελληνική τεκμηρίωση ώστε το project να παρουσιάζεται ολοκληρωμένα ως ερευνητική ή πανεπιστημιακή εργασία.

Συγκεκριμένα:

- εξηγήθηκε το πρόβλημα Return-to-Home υπό αβεβαιότητα,
- περιγράφηκε γιατί ένα σταθερό όριο μπαταρίας δεν είναι αρκετό,
- αναλύθηκαν οι deterministic, risk-aware, adaptive και health-aware πολιτικές,
- τεκμηριώθηκε η Monte Carlo εκτίμηση safe return probability,
- περιγράφηκαν battery uncertainty, battery aging και wind gust experiments,
- παρουσιάστηκαν οι βασικές μετρικές αξιολόγησης,
- καταγράφηκαν τα σημαντικότερα αποτελέσματα,
- προστέθηκε ελληνικό project report στο `docs/PROJECT_REPORT_EL.md`,
- προστέθηκε ελληνικός οδηγός εγκατάστασης στο `docs/INSTALLATION_EL.md`,
- δημιουργήθηκε PDF report για χρήση σε παράδοση ή παρουσίαση.

## Συμπέρασμα

Το project δείχνει ότι οι deterministic RTH πολιτικές δεν επαρκούν σε safety-critical UAV σενάρια. Οι Monte Carlo risk-aware πολιτικές επιτρέπουν πιο τεκμηριωμένη λήψη αποφάσεων, επειδή λαμβάνουν υπόψη την αβεβαιότητα. Υπάρχει όμως σαφές trade-off: περισσότερη ασφάλεια σημαίνει συχνότερο RTH και μικρότερο mission completion.
