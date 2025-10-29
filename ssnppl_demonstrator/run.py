#!/usr/bin/env python3
import subprocess
import time
import threading
import sys

BINARY = "./ssnppl_demonstrator"
LOGFILE = "output.log"
TIMEOUT = 10  # seconds

def run_watchdog():
    while True:
        # Start the binary process
        with subprocess.Popen(
            [BINARY],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            universal_newlines=True
        ) as proc:

            last_spartn_time = time.time()

            # Open log file
            with open(LOGFILE, "a") as log_file:

                # Function to read stdout line by line
                def reader():
                    nonlocal last_spartn_time
                    for line in proc.stdout:
                        line = line.rstrip()
                        print(line)
                        log_file.write(line + "\n")
                        log_file.flush()

                        # Check for SPARTN message
                        if "New SPARTN Serial Message" in line:
                            last_spartn_time = time.time()

                t = threading.Thread(target=reader, daemon=True)
                t.start()

                # Watchdog loop
                while proc.poll() is None:
                    time.sleep(1)
                    if time.time() - last_spartn_time > TIMEOUT:
                        print(f"No SPARTN messages for {TIMEOUT}s. Restarting...")
                        proc.terminate()
                        try:
                            proc.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            proc.kill()
                            proc.wait()
                        break  # restart outer loop

        print("Restarting binary in 1 second...")
        time.sleep(1)

if __name__ == "__main__":
    run_watchdog()
