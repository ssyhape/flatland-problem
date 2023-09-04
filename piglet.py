from lib_piglet.cli.cli_tool import *
from lib_piglet.cli.run_tool import *
import os



def main():

    args = parse_args()
    if args.problem == None and sys.stdin.isatty():
        print("err; You must provide a problem scenario file or provide problem through standard input", file = sys.stderr)
        print("piglet.py -h for help", file=sys.stderr)
        exit(1)

    header_readed = False

    # detect which source to accept scenario data
    if not sys.stdin.isatty():
        source = sys.stdin
    else:
        if not os.path.exists(args.problem):
            print("err; Given problem scenario file does not exist: {}".format(args.problem), file = sys.stderr)
            exit(1)
        source = open(args.problem)

    print_header(args.anytime)
    if args.output_file:
        out = open(args.output_file, "w+")
        out.write(csv_header(args.anytime))

    domain_type = None
    multi_tasks = []
    problem_amount = 0
    for line in source:
        if problem_amount >=  args.problem_number:
            break
        content = line.strip().split()
        if len(content) == 0 or content[0] == "#" or content[0] == "c":
            continue

        if not header_readed:
            domain_type = parse_scen_header(content)
            header_readed = True
            continue

        task = parse_problem(content, domain_type)

        if args.multi_agent:
            multi_tasks.append(task)
            search = run_multi_tasks(domain_type,multi_tasks,args)
        else:
            search = run_task(task, args)
        problem_amount += 1
        print(statistic_string(args,search,args.anytime))
        if args.output_file:
            out.write(statistic_csv(args,search,args.anytime))

    if args.output_file:
        out.close()





        
if __name__ == "__main__":
    main()



