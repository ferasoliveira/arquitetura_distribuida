# -*- coding: utf-8 -*-
"""
smoke_test.py — Smoke Test de Integração do Benchmark
======================================================
TCC Fernando Oliveira — Passo 3: Ambiente Virtual, Pontes e Simulação
"""

import os
import sys
import time
import subprocess
import csv
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("smoke_test")

WEBOTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

def run_smoke_test() -> bool:
    logger.info("=== Iniciando Smoke Test de Integração ===")
    
    benchmark_script = os.path.join(WEBOTS_DIR, "benchmark", "benchmark.py")
    output_csv = os.path.join(WEBOTS_DIR, "output", "benchmark_move_j_short_eb15.csv")
    
    # Remove CSV antigo se existir para garantir validação limpa
    if os.path.exists(output_csv):
        try:
            os.remove(output_csv)
            logger.info("CSV antigo removido: %s", output_csv)
        except OSError:
            pass
            
    # Executa o benchmark para a trajetória curta move_j_short no target eb15
    cmd = [
        sys.executable,
        benchmark_script,
        "--target", "eb15",
        "--trajectory", "move_j_short",
        "--headless"
    ]
    
    logger.info("Executando benchmark: %s", " ".join(cmd))
    
    try:
        # Executa com timeout de 90 segundos (60s de boot + 30s de execução)
        result = subprocess.run(
            cmd,
            cwd=WEBOTS_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=90.0
        )
        
        logger.info("Benchmark finalizado com exit code: %d", result.returncode)
        
        if result.returncode != 0:
            logger.error("Falha no benchmark (exit code não zero).")
            logger.error("STDOUT:\n%s", result.stdout)
            logger.error("STDERR:\n%s", result.stderr)
            return False
            
        # Verifica se o arquivo CSV foi criado
        if not os.path.exists(output_csv):
            logger.error("Arquivo CSV de saída não foi gerado em %s", output_csv)
            return False
            
        # Valida conteúdo do CSV
        logger.info("Verificando consistência do CSV gerado...")
        with open(output_csv, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader, None)
            
            if not header:
                logger.error("Arquivo CSV está vazio.")
                return False
                
            # Verifica colunas essenciais
            required_cols = ["timestamp", "source", "sequence", "q1", "q2", "q3", "q4", "q5", "q6"]
            for col in required_cols:
                if col not in header:
                    logger.error("Coluna essencial ausente no CSV: %s", col)
                    return False
                    
            rows = list(reader)
            logger.info("CSV contém %d linhas de telemetria.", len(rows))
            if len(rows) < 5:
                logger.error("Número de linhas de telemetria no CSV é suspeitamente baixo (%d).", len(rows))
                return False
                
        logger.info("=== Smoke Test passou com sucesso! ✓ ===")
        return True
        
    except subprocess.TimeoutExpired:
        logger.error("Erro: Timeout de execução do smoke test expirado (> 40s).")
        return False
    except Exception as e:
        logger.error("Erro inesperado no smoke test: %s", e)
        return False

if __name__ == "__main__":
    success = run_smoke_test()
    sys.exit(0 if success else 1)
