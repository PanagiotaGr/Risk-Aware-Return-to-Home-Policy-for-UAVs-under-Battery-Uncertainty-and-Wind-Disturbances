# Αναλυτική Τεκμηρίωση Project

**Project:** Risk-Aware Return-to-Home Policy for UAVs under Battery Uncertainty and Wind Disturbances

## 1. Σκοπός

Το project μελετά πολιτικές Return-to-Home για UAVs όταν υπάρχει αβεβαιότητα στην μπαταρία, battery aging και δυναμικός άνεμος. Αντί για απλό σταθερό όριο μπαταρίας, το σύστημα εκτιμά πιθανοτικά αν το UAV μπορεί να επιστρέψει με ασφάλεια.

## 2. Ερευνητικό ερώτημα

Το βασικό ερώτημα είναι αν probabilistic και health-aware RTH πολιτικές μπορούν να βελτιώσουν την ασφάλεια επιστροφής σε σχέση με deterministic threshold-based triggering.

## 3. Πολιτικές που συγκρίνονται

- deterministic_threshold: ενεργοποιεί RTH όταν το εκτιμώμενο SoC πέσει κάτω από σταθερό όριο.
- risk_aware_mc: χρησιμοποιεί Monte Carlo sampling για εκτίμηση πιθανότητας ασφαλούς επιστροφής.
- adaptive_risk_mc: προσαρμόζει δυναμικά το risk threshold με βάση αβεβαιότητα, άνεμο, απόσταση και battery health.
- health_aware_risk_mc: λαμβάνει ρητά υπόψη την υποβάθμιση της μπαταρίας.

## 4. Κεντρικός κανόνας απόφασης

Ο planner εκτιμά την πιθανότητα ασφαλούς επιστροφής. Αν η πιθανότητα είναι μικρότερη από το threshold tau, τότε ενεργοποιείται RTH. Στη health-aware εκδοχή η πιθανότητα εξαρτάται και από τον παράγοντα battery health.

## 5. Monte Carlo risk estimation

Η Monte Carlo διαδικασία δειγματοληπτεί πιθανές τιμές SoC, ανέμου και extra drain. Για κάθε δείγμα υπολογίζεται αν η διαθέσιμη ενέργεια επαρκεί για επιστροφή. Το ποσοστό επιτυχών δειγμάτων αποτελεί την εκτίμηση της πιθανότητας safe return.

## 6. Πειραματικό πλαίσιο

Το script `experiments/run_monte_carlo_experiments.py` είναι standalone και αναπαράγει τα βασικά πειράματα χωρίς ROS 2 runtime. Παράγει CSV αρχεία και plots για ανάλυση πολιτικών, σεναρίων, energy margins και mission completion.

## 7. Βασικές μετρικές

- RTH trigger rate
- safe return rate
- failure rate
- early RTH rate
- mission completion rate
- mean energy margin
- negative margin rate
- mean battery left

## 8. Κύρια αποτελέσματα

| Policy | RTH Trigger | Safe Return | Failure | Mission Completion |
| --- | ---: | ---: | ---: | ---: |
| deterministic_threshold | 0.15% | 0.15% | 20.54% | 99.47% |
| risk_aware_mc | 40.33% | 19.94% | 20.83% | 78.79% |
| adaptive_risk_mc | 52.23% | 31.00% | 21.27% | 73.31% |
| health_aware_risk_mc | 39.90% | 19.90% | 20.75% | 79.04% |

Η deterministic πολιτική διατηρεί υψηλό mission completion επειδή σχεδόν δεν ενεργοποιεί RTH. Όμως έχει πολύ χαμηλή συμπεριφορά safe return. Οι Monte Carlo πολιτικές αυξάνουν σημαντικά το safe return, με κόστος μικρότερο mission completion.

## 9. Battery aging

Η μείωση battery health αυξάνει έντονα το ρίσκο επιστροφής. Η health-aware πολιτική είναι πιο κατάλληλη σε degraded battery conditions, επειδή λαμβάνει υπόψη ότι το ονομαστικό SoC δεν αντιστοιχεί πάντα στην ίδια πραγματική διαθέσιμη ενέργεια.

## 10. Dynamic wind gusts

Σε δυναμικές ριπές ανέμου οι Monte Carlo πολιτικές αυξάνουν σημαντικά το safe return σε σχέση με τη deterministic πολιτική. Σε high και extreme gust scenarios η επιστροφή γίνεται δυσκολότερη για όλες τις πολιτικές.

## 11. Ablation studies

Τα ablation studies δείχνουν ότι το risk threshold tau ελέγχει το επίπεδο συντηρητικότητας. Μεγαλύτερο tau οδηγεί σε περισσότερα RTH triggers. Επίσης, 50 έως 100 Monte Carlo samples φαίνεται να είναι συχνά αρκετά για σταθερή απόφαση, ενώ περισσότερα samples αυξάνουν το runtime.

## 12. Οδηγίες εκτέλεσης

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install "numpy<2" matplotlib
python3 experiments/run_monte_carlo_experiments.py --trials 200 --mc-samples 500
python3 experiments/run_ablation_studies.py --trials 200
python3 experiments/run_battery_aging_study.py --trials 200
python3 experiments/run_wind_gust_study.py --trials 200
```

## 13. Συμπέρασμα

Το project δείχνει ότι οι deterministic threshold-based RTH πολιτικές δεν επαρκούν για safety-critical UAV αποφάσεις υπό αβεβαιότητα. Οι Monte Carlo risk-aware πολιτικές είναι πιο τεκμηριωμένες, επειδή εκτιμούν άμεσα την πιθανότητα ασφαλούς επιστροφής. Η adaptive και η health-aware πολιτική αυξάνουν την ασφάλεια σε δύσκολες συνθήκες, αλλά μειώνουν την αποδοτικότητα της αποστολής.
