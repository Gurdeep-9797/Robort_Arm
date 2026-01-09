import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import path from 'path';

export default defineConfig({
    plugins: [react()],
    build: {
        outDir: '../data',
        emptyOutDir: false,
        rollupOptions: {
            output: {
                // Keep bundle size under 500KB for ESP32 SPIFFS
                manualChunks: {
                    'vendor-react': ['react', 'react-dom'],
                    'vendor-three': ['three', '@react-three/fiber', '@react-three/drei'],
                    'vendor-flow': ['reactflow'],
                },
            },
        },
        chunkSizeWarningLimit: 400,
    },
    resolve: {
        alias: {
            '@': path.resolve(__dirname, './src'),
        },
    },
    server: {
        proxy: {
            '/api': {
                target: 'http://192.168.1.100', // ESP32 IP (update as needed)
                changeOrigin: true,
            },
            '/ws': {
                target: 'ws://192.168.1.100',
                ws: true,
            },
        },
    },
});
