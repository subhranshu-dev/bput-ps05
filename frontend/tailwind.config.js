/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{ts,tsx}'],
  theme: {
    extend: {
      fontFamily: {
        sans: ['Inter', 'system-ui', 'sans-serif'],
        mono: ['"JetBrains Mono"', 'ui-monospace', 'monospace'],
      },
      colors: {
        panel: '#0d1520',
        panel2: '#111c2a',
        edge: '#1e2d40',
        ink: '#d7e3f0',
        dim: '#7e93ab',
        accent: '#38bdf8',
        ok: '#34d399',
        warn: '#fbbf24',
        bad: '#f87171',
      },
    },
  },
  plugins: [],
}
