import { defineConfig } from 'vite';

export default defineConfig({
  server: {
    port: 5173,
    open: true,
    host: '0.0.0.0'
  },
  build: {
    target: 'esnext',
    chunkSizeWarningLimit: 2000
  }
});
