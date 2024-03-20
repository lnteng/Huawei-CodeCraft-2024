#!/bin/bash

map="./maps/map-3.8.txt"  # 指定你的map
main="./build/main"
output_file="./replay/output.txt"
num_iterations=20 # 循环执行次数
total_scores=0
declare -A score_map  # 声明一个关联数组来保存随机数和对应的分数

# 清空输出文件
> "$output_file"

echo "Starting PreliminaryJudge script..."

echo ""
echo "Processing map: $map"
scores=0


for ((i=1; i<=$num_iterations; i++)); do
    if [ -f "$map" ]; then
        number=$RANDOM  # 使用随机数种子替代数组中的元素

        # 执行PreliminaryJudge命令，并将输出重定向到output_file
        ./PreliminaryJudge "$main" -s "$number" -l NONE -m "$map" >> "$output_file" 2>&1
        
        second_last_line=$(tail -n 2 "$output_file" | head -n 1)
        score=$(echo "$second_last_line" | grep -oP '"score":\K\d+')
        ((scores += score))
        echo "Score $map for iteration $i: $score"
        score_map["$number"]=$score
    fi
done

echo "Map processed: $map"
((average_score = scores / num_iterations))
echo "Average score $map for iteration: $average_score"

# 打印出分数对应的随机数种子
echo "Top scores seed:" >> "$output_file"
for key in "${!score_map[@]}"; do
    echo "$key: ${score_map[$key]}" | sort -rn -k2 | head -n1
    echo "$key: ${score_map[$key]}" | sort -rn -k2 | head -n3 >> "$output_file"
done




