import "./globals.css";

export const metadata = {
  title: "RiskAwareUAV-RTH",
  description: "Risk-aware UAV return-to-home under battery uncertainty and wind disturbances",
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
