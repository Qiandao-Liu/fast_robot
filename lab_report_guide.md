Lab report writing rules for MAE 4190 Fast Robots. Apply to all labs.

The report should explain how you did something, not just prove you did it. Imaging you are a American college student: casual, concise, honest and hardwork. Skip unnecessary words. Imperfect grammar is encouraged and totaly fine as long as the technical understanding is deep clear.

WRITING RULES

Check the words limit requirement in lab.md, normally the word limit is around <800 to <1500 for the whole report.

Before editing or publishing any report page, run `ble_robot_1.4/scripts/fix_report_videos.sh` on the matching image folder so every video used by the page is converted to web-safe `.mp4`.

Example command: `bash /Users/qiandaoliu/fastrobot/ble_robot_1.4/scripts/fix_report_videos.sh /Users/qiandaoliu/fastrobot/Qiandao-Liu.github.io/images/mae4190/lab5`

After conversion, the webpage must reference only `.mp4` files inside `<video>` tags. Do not leave `.mov` or `.MOV` links in the markdown.

Do not use bold for emphasis. Plain prose is preferred. Only use backtick formatting for actual code identifiers.

Do not use dashes (em dash —, or hyphen-dash " - ") to connect two clauses or to explain something inline. Write a separate sentence instead. Use because / since / which / therefore / so to join related ideas. A colon is fine for introducing a list or elaboration.

Do not use parentheses for supplementary explanations. Either work the information into the sentence or leave it out.

Section headings use descriptive names only. Do not prefix with "Task N:" — that is lab spec language, not report structure.

Do not use horizontal rules between sections. Headings already provide visual separation.

CODE SNIPPETS

Every task that touches Arduino code must show the relevant Arduino snippet. Every task that touches Python must show the relevant Python snippet. If a task involves both sides, show both.

Only show the directly relevant function or case block. Skip includes, global declarations, and unrelated setup.

All code snippets go inside a collapsible details block using this exact format (required for Jekyll kramdown rendering):

<details>
<summary>Arduino: brief description</summary>
<div markdown="1">

```cpp
// code here
```

</div>
</details>

Narrative text (explanation, results, discussion) stays outside the details block.

RESULTS AND DISCUSSION

Quantify every result. Do not just say "it worked" — say what value was received, what rate was measured, what error was observed.

Discussion should explain why, not just what. Reference actual measured numbers to support conclusions. If something unexpected happened or a bug was found, write it up.

Things that always require code (not just a screenshot or one-line description): notification handler registration and callback logic, sampling/sending logic inside loop(), array definition and overflow guard, use of millis().

END OF REPORT

End every lab report with 1-2 cat photos from /images/mae4190/cats/ and the line: Meet my cat Mulberry! 🐱
