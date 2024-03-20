#!/bin/bash

maps="./maps"
main="./build/main"
output_file="./replay/output.txt"
num_iterations=5 # 循环执行五次
map_counter=0
total_scores=0
numbers=(12345 67890 11121 31415 92653 1)  # 定义你的数组

# 清空输出文件
> "$output_file"

echo "Starting PreliminaryJudge script..."

for map in "$maps"/*; do
    echo ""
    echo "Processing map: $map"
    scores=0

    for ((i=1; i<=$num_iterations; i++)); do
        if [ -f "$map" ]; then
            number=${numbers[$i-1]}  # 使用数组中的元素替代随机数

            # 执行PreliminaryJudge命令，并将输出重定向到output_file
            ./PreliminaryJudge "$main" -s "$number" -l NONE -m "$map" >> "$output_file" 2>&1
            
            second_last_line=$(tail -n 2 "$output_file" | head -n 1)
            score=$(echo "$second_last_line" | grep -oP '"score":\K\d+')
            ((scores += score))
            echo "Score $map for iteration $i: $score"
        fi
    done

    echo "Map processed: $map"
    ((average_score = scores / num_iterations))
    echo "Average score $map for iteration: $average_score"
    ((map_counter += 1))
    ((total_scores += scores))
done

echo "PreliminaryJudge script finished."

((total_average_score = total_scores / map_counter / num_iterations))
echo "Avergae scores for all $map_counter maps: $total_average_score"