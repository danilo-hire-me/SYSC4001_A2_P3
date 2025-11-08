/**
 * @file interrupts.cpp
 * @author S.
 */

#include "interrupts_danilobukvic_oluwatobiolowookere.hpp"
#include <sstream>

static unsigned int next_pid = 1;

std::tuple<std::string, std::string, int>
simulate_trace(std::vector<std::string> trace_file,
               int start_time,
               std::vector<std::string> vectors,
               std::vector<int> delays,
               std::vector<external_file> external_files,
               PCB current,
               std::vector<PCB> wait_queue)
{
    std::ostringstream exec_log;     // collects execution steps
    std::ostringstream status_log;   // collects system status snapshots
    int now = start_time;

    for (size_t idx = 0; idx < trace_file.size(); ++idx) {
        const std::string& line = trace_file[idx];
        auto [activity, duration_intr, program_name] = parse_trace(line);

        if (activity == "CPU") {
            exec_log << now << ", " << duration_intr << ", CPU Burst\n";
            now += duration_intr;
        }
        else if (activity == "SYSCALL") {
            // kernel entry + vector look-up + ISR address fetch
            exec_log << now << ", 1, Switch to kernel mode\n";   now += 1;
            exec_log << now << ", 4, context saved\n";           now += 4;
            exec_log << now << ", 1, find vector " << duration_intr
                    << " in memory " << vectors[duration_intr] << "\n"; now += 1;
            exec_log << now << ", 1, obtain ISR address\n";      now += 1;

            // split device time into three segments (driver/check/instruction)
            int remain = delays[duration_intr];

            int t1 = rand() % remain - 2;
            exec_log << now << ", " << t1 << ", Call device driver\n";
            now += t1; remain -= t1;

            int t2 = rand() % remain - 1;
            exec_log << now << ", " << t2 << ", Perform device check\n";
            now += t2; remain -= t2;

            int t3 = remain;
            exec_log << now << ", " << t3 << ", Send device instruction\n";
            now += t3; remain -= t3;

            exec_log << now << ", 1, IRET\n";
            now += 1;
        }
        else if (activity == "END_IO") {
            // kernel entry + vector + storage/reset/standby
            exec_log << now << ", 1, switch to kernel mode\n";   now += 1;
            exec_log << now << ", 4, context saved\n";           now += 4;
            exec_log << now << ", 1, find vector " << duration_intr
                    << " in memory " << vectors[duration_intr] << "\n"; now += 1;

            int remain = delays[duration_intr];

            int t1 = rand() % remain - 2;
            exec_log << now << ", " << t1 << ", store information in memory\n";
            now += t1; remain -= t1;

            int t2 = rand() % remain - 1;
            exec_log << now << ", " << t2 << ", reset the io operation\n";
            now += t2; remain -= t2;

            int t3 = remain;
            exec_log << now << ", " << t3 << ", Send standby instruction\n";
            now += t3; remain -= t3;

            exec_log << now << ", 1, IRET\n";
            now += 1;
        }
        else if (activity == "FORK") {
            // common interrupt boilerplate
            auto [intr_txt, t_after] = intr_boilerplate(now, 2, 10, vectors);
            exec_log << intr_txt;
            now = t_after;

            // clone PCB (parent → child)
            exec_log << now << ", " << duration_intr << ", cloning the PCB\n";
            now += duration_intr;

            PCB child = current;
            child.PID = next_pid++;
            child.partition_number = -1;

            // attempt to allocate memory for child
            if (!allocate_memory(&child)) {
                exec_log << now << ", 0, FORK failed: No memory for child process\n";
                exec_log << now << ", 1, IRET\n";
                now += 1;

                // find where parent should continue
                int parent_pos = -1, endif_pos = -1;
                for (size_t j = idx + 1; j < trace_file.size(); ++j) {
                    auto [act_j, _d, _pn] = parse_trace(trace_file[j]);
                    if (act_j == "IF_PARENT") parent_pos = int(j);
                    else if (act_j == "ENDIF") {
                        endif_pos = int(j);
                        if (parent_pos == -1) parent_pos = int(j);
                        break;
                    }
                }
                if (parent_pos != -1)       idx = size_t(parent_pos);
                else if (endif_pos != -1)   idx = size_t(endif_pos);
                else                        idx = trace_file.size(); // bail if malformed
                continue;
            }

            // parent goes to wait queue; child becomes current
            PCB parent_snapshot = current;
            wait_queue.push_back(parent_snapshot);
            current = child;

            // scheduler + IRET
            exec_log << now << ", 0, scheduler called\n";
            now += 1;
            status_log << "time: " << now << "; current trace: " << trace_file[idx] << "\n"
                       << print_PCB(current, wait_queue) << "\n";
            exec_log << now << ", 1, IRET\n";

            // collect child-only trace and track parent resume point
            std::vector<std::string> child_trace;
            bool muted = true;
            int parent_pos = -1, endif_pos = -1;

            for (size_t j = idx + 1; j < trace_file.size(); ++j) {
                auto [act_j, _d, _pn] = parse_trace(trace_file[j]);

                if (act_j == "IF_CHILD") { muted = false; continue; }
                if (act_j == "IF_PARENT") { muted = true; parent_pos = int(j); continue; }
                if (act_j == "ENDIF") {
                    muted = false;
                    endif_pos = int(j);
                    if (parent_pos == -1) parent_pos = int(j);
                    continue;
                }
                if (!muted) child_trace.push_back(trace_file[j]);
            }

            if (parent_pos != -1)      idx = size_t(parent_pos);
            else if (endif_pos != -1)  idx = size_t(endif_pos);

            // run child recursively
            PCB parent_pcb = wait_queue.back();
            auto [child_exec, child_stat, child_done] =
                simulate_trace(child_trace, now, vectors, delays, external_files, current, wait_queue);

            exec_log << child_exec.str();
            status_log << child_stat;
            now = child_done;

            // resume parent
            wait_queue.pop_back();
            current = parent_pcb;
        }
        else if (activity == "EXEC") {
            // common interrupt boilerplate
            auto [intr_txt, t_after] = intr_boilerplate(now, 3, 10, vectors);
            now = t_after;
            exec_log << intr_txt;

            // replace current image: release memory first (if any)
            if (current.partition_number != -1) {
                free_memory(&current);
            }

            // look up new image size; log the search duration from trace
            unsigned int new_size = get_size(program_name, external_files);
            exec_log << now << ", " << duration_intr
                     << ", Program is " << new_size << " Mb large\n";
            now += duration_intr;

            // (re)allocate memory for the new image
            current.program_name = program_name;
            current.size = new_size;
            current.partition_number = -1;

            if (!allocate_memory(&current)) {
                exec_log << now << ", 0, EXEC failed: Memory allocation failed for " << program_name << "\n";
                status_log << "time: " << now << "; current trace: " << trace_file[idx] << "\n"
                           << print_PCB(current, wait_queue) << "\n";
                return {exec_log.str(), status_log.str(), now};
            }

            // loader, marking, pcb update
            int load_ms = static_cast<int>(new_size) * 15;
            exec_log << now << ", " << load_ms << ", loading program into memory\n";
            now += load_ms;

            exec_log << now << ", 3, marking partition as occupied\n"; now += 3;
            exec_log << now << ", 6, updating PCB\n";                  now += 6;

            // schedule + return
            exec_log << now << ", 0, scheduler called\n";
            exec_log << now << ", 1, IRET\n";                          now += 1;

            status_log << "time: " << now << "; current trace: " << trace_file[idx] << "\n"
                       << print_PCB(current, wait_queue) << "\n";

            // run external trace (the new program)
            std::ifstream exec_trace_file(program_name + ".txt");
            std::vector<std::string> exec_traces;
            std::string subline;
            while (std::getline(exec_trace_file, subline)) {
                exec_traces.push_back(subline);
            }

            auto [sub_exec, sub_status, new_now] =
                simulate_trace(exec_traces, now, vectors, delays, external_files, current, wait_queue);

            exec_log << sub_exec;
            status_log << sub_status;
            now = new_now;

            // per original logic
            break;
        }
    }

    // release whatever is left allocated for the finishing process
    if (current.partition_number != -1) {
        free_memory(&current);
    }

    return {exec_log.str(), status_log.str(), now};
}

int main(int argc, char** argv)
{
    auto [vectors, delays, external_files] = parse_args(argc, argv);
    std::ifstream input_file(argv[1]);

    // quick inventory print
    print_external_files(external_files);

    // bootstrap PCB (no partition yet)
    PCB current(0, -1, "init", 1, -1);
    if (!allocate_memory(&current)) {
        std::cerr << "ERROR! Memory allocation failed!" << std::endl;
    }

    std::vector<PCB> wait_queue;

    // load trace file into memory
    std::vector<std::string> trace_lines;
    std::string ln;
    while (std::getline(input_file, ln)) {
        trace_lines.push_back(ln);
    }
    input_file.close();

    auto [exec_out, status_out, _t] =
        simulate_trace(trace_lines, 0, vectors, delays, external_files, current, wait_queue);

    write_output(exec_out, "execution.txt");
    write_output(status_out, "system_status.txt");
    return 0;
}

