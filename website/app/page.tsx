"use client";

import { motion } from "framer-motion";

const sections = [
  {
    title: "Research question",
    body: "How can a UAV decide when to return home under uncertain battery state, stochastic wind, and safety-critical mission constraints?",
  },
  {
    title: "Architecture",
    body: "YAML configs drive a 2D simulator, uncertainty models, Monte Carlo safe-return estimation, RTH policies, metrics, and generated figures.",
  },
  {
    title: "Experiments",
    body: "Controlled scenarios cover battery uncertainty, headwind, tailwind, crosswind, gusts, wind variance, mission length, and threshold sensitivity.",
  },
  {
    title: "Honest status",
    body: "Simulation software with explicit implemented/prototype/planned separation. No real-flight safety claim is made without validation.",
  },
];

export default function Home() {
  return (
    <main className="min-h-screen bg-slate-950 text-slate-100">
      <section className="mx-auto grid max-w-6xl gap-10 px-6 py-24 lg:grid-cols-[1.05fr_0.95fr] lg:items-center">
        <motion.div initial={{ opacity: 0, y: 16 }} animate={{ opacity: 1, y: 0 }}>
          <p className="mb-4 text-sm uppercase tracking-[0.3em] text-cyan-300">RiskAwareUAV-RTH</p>
          <h1 className="text-5xl font-semibold leading-tight">
            Risk-aware return-to-home for UAVs under battery uncertainty and wind disturbances
          </h1>
          <p className="mt-6 max-w-3xl text-lg text-slate-300">
            A reproducible simulation research project studying when a UAV should return home by estimating the probability that return remains energetically feasible.
          </p>
          <div className="mt-8 flex flex-wrap gap-4">
            <a className="rounded-xl bg-cyan-400 px-5 py-3 font-medium text-slate-950" href="https://github.com/PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances">GitHub</a>
            <a className="rounded-xl border border-slate-600 px-5 py-3" href="https://github.com/PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances/blob/main/docs/MATHEMATICAL_FORMULATION.md">Mathematical formulation</a>
          </div>
        </motion.div>
        <motion.div initial={{ opacity: 0, scale: 0.98 }} animate={{ opacity: 1, scale: 1 }} className="rounded-3xl border border-slate-800 bg-slate-900/70 p-4 shadow-2xl">
          <img src="/demo.gif" alt="Code-generated risk-aware UAV RTH demo" className="w-full rounded-2xl" />
          <p className="mt-3 text-sm text-slate-400">Code-generated visualization: trajectory, wind, battery, safe-return probability, threshold, and RTH trigger.</p>
        </motion.div>
      </section>

      <section className="mx-auto grid max-w-6xl gap-4 px-6 pb-20 md:grid-cols-2">
        {sections.map((item) => (
          <div key={item.title} className="rounded-2xl border border-slate-800 bg-slate-900/70 p-6">
            <h2 className="text-xl font-semibold">{item.title}</h2>
            <p className="mt-3 text-slate-400">{item.body}</p>
          </div>
        ))}
      </section>

      <section className="mx-auto grid max-w-6xl gap-4 px-6 pb-24 lg:grid-cols-2">
        <div className="rounded-2xl border border-slate-800 bg-slate-900 p-6">
          <h2 className="text-2xl font-semibold">Decision rule</h2>
          <p className="mt-4 text-slate-300">Trigger RTH when estimated safe-return probability falls below a configurable threshold τ.</p>
          <pre className="mt-4 overflow-x-auto rounded-xl bg-slate-950 p-4 text-cyan-200">P_safe = P(E_required &lt; E_available | x_t, SoC_t, w_t)</pre>
        </div>
        <div className="rounded-2xl border border-slate-800 bg-slate-900 p-6">
          <h2 className="text-2xl font-semibold">Reproducibility</h2>
          <p className="mt-4 text-slate-300">Every reported number should be tied to a YAML config, random seed, command, and generated CSV artifact.</p>
          <pre className="mt-4 overflow-x-auto rounded-xl bg-slate-950 p-4 text-cyan-200">python scripts/run_all_experiments.py --output-dir results/controlled</pre>
        </div>
      </section>
    </main>
  );
}
