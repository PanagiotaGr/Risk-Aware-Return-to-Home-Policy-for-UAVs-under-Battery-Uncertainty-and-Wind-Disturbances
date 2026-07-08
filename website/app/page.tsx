"use client";

import { motion } from "framer-motion";

const sections = [
  "Risk-aware autonomy",
  "Battery uncertainty",
  "Wind-aware energy modelling",
  "Monte Carlo safe-return estimation",
  "Honest implemented/prototype/planned roadmap",
];

export default function Home() {
  return (
    <main className="min-h-screen bg-slate-950 text-slate-100">
      <section className="mx-auto max-w-5xl px-6 py-24">
        <motion.div initial={{ opacity: 0, y: 16 }} animate={{ opacity: 1, y: 0 }}>
          <p className="mb-4 text-sm uppercase tracking-[0.3em] text-cyan-300">RiskAwareUAV-RTH</p>
          <h1 className="text-5xl font-semibold leading-tight">
            Risk-aware return-to-home for UAVs under battery uncertainty and wind disturbances
          </h1>
          <p className="mt-6 max-w-3xl text-lg text-slate-300">
            A reproducible simulation research project studying when a UAV should return home by estimating the probability that return remains energetically feasible.
          </p>
          <div className="mt-8 flex gap-4">
            <a className="rounded-xl bg-cyan-400 px-5 py-3 font-medium text-slate-950" href="../README.md">Read repository</a>
            <a className="rounded-xl border border-slate-600 px-5 py-3" href="https://github.com/PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances">GitHub</a>
          </div>
        </motion.div>
      </section>

      <section className="mx-auto grid max-w-5xl gap-4 px-6 pb-20 md:grid-cols-2">
        {sections.map((item) => (
          <div key={item} className="rounded-2xl border border-slate-800 bg-slate-900/70 p-6">
            <h2 className="text-xl font-semibold">{item}</h2>
            <p className="mt-3 text-slate-400">Simulation-grade module with explicit limitations and reproducible configuration.</p>
          </div>
        ))}
      </section>

      <section className="mx-auto max-w-5xl px-6 pb-24">
        <div className="rounded-2xl border border-slate-800 bg-slate-900 p-6">
          <h2 className="text-2xl font-semibold">Decision rule</h2>
          <p className="mt-4 text-slate-300">Trigger RTH when estimated safe-return probability falls below a configurable threshold τ.</p>
          <pre className="mt-4 overflow-x-auto rounded-xl bg-slate-950 p-4 text-cyan-200">P_safe = P(E_required &lt; E_available | x_t, SoC_t, w_t)</pre>
        </div>
      </section>
    </main>
  );
}
