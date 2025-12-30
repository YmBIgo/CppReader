import fs from "fs/promises";

/**
 * C/C++ 共通の「文字列・コメントを無視した走査」用の状態
 */
type ScanState = {
  inLineComment: boolean;
  inBlockComment: boolean;
  inString: boolean; // "
  inChar: boolean; // '
  inRawString: boolean; // R"delim( ... )delim"
  rawDelim: string;
  escape: boolean;
};

function createScanState(): ScanState {
  return {
    inLineComment: false,
    inBlockComment: false,
    inString: false,
    inChar: false,
    inRawString: false,
    rawDelim: "",
    escape: false,
  };
}

/**
 * 1行をスキャンして、コメント/文字列を無視しながら
 * '{' '}' の増減、raw string の開始/終了などを状態遷移する
 */
function scanLineForBraces(line: string, state: ScanState): { open: number; close: number } {
  let open = 0;
  let close = 0;

  const len = line.length;
  for (let i = 0; i < len; i++) {
    const c = line[i];
    const next = i + 1 < len ? line[i + 1] : "";

    // 行コメントは行末で終わる
    if (state.inLineComment) break;

    // block comment 中
    if (state.inBlockComment) {
      if (c === "*" && next === "/") {
        state.inBlockComment = false;
        i++;
      }
      continue;
    }

    // raw string 中:  )delim"
    if (state.inRawString) {
      // raw string 終端の候補: ) + rawDelim + "
      if (c === ")") {
        const tail = line.slice(i + 1);
        const endMark = state.rawDelim + `"`;
        if (tail.startsWith(endMark)) {
          state.inRawString = false;
          state.rawDelim = "";
          i += endMark.length; // ここで i++ もされるので実際は少し進みすぎないように
        }
      }
      continue;
    }

    // string / char 中
    if (state.inString) {
      if (!state.escape && c === '"') state.inString = false;
      state.escape = !state.escape && c === "\\";
      continue;
    }
    if (state.inChar) {
      if (!state.escape && c === "'") state.inChar = false;
      state.escape = !state.escape && c === "\\";
      continue;
    }

    // ここから「通常コード」

    // コメント開始
    if (c === "/" && next === "/") {
      state.inLineComment = true;
      break;
    }
    if (c === "/" && next === "*") {
      state.inBlockComment = true;
      i++;
      continue;
    }

    // raw string 開始: R"delim(
    if (c === "R" && next === '"') {
      // R" ... (
      // 例: R"(....)" / R"tag(....)tag"
      const rest = line.slice(i + 2); // after R"
      const parenIdx = rest.indexOf("(");
      if (parenIdx !== -1) {
        const delim = rest.slice(0, parenIdx); // may be ""
        state.inRawString = true;
        state.rawDelim = delim;
        // i を '(' の位置まで進める（ただし for ループで i++ される）
        i = i + 2 + parenIdx; // points at '('
        continue;
      }
    }

    // string / char 開始
    if (c === '"') {
      state.inString = true;
      state.escape = false;
      continue;
    }
    if (c === "'") {
      state.inChar = true;
      state.escape = false;
      continue;
    }

    // brace カウント
    if (c === "{") open++;
    else if (c === "}") close++;
  }

  // 行末で line comment 終了
  state.inLineComment = false;

  return { open, close };
}

/**
 * C/C++ の #define 継続行（\）を読む
 */
function readDefineBlock(lines: string[], startLine: number, maxLookahead = 80): { endLine: number } {
  let endLine = startLine;
  for (let i = startLine; i < Math.min(lines.length, startLine + maxLookahead); i++) {
    endLine = i;
    const l = lines[i];
    // バックスラッシュ継続
    if (!l.trimEnd().endsWith("\\")) break;
  }
  return { endLine };
}

/**
 * C++ で「関数定義っぽい開始行」か判定を少し強くする
 * - 行や付近に '(' がある
 * - 近傍に '{' がある（本体が近い）
 * - class/namespace/enum の開始は除外気味
 */
function looksLikeFunctionStart(windowText: string): boolean {
  const t = windowText;
  if (!t.includes("(")) return false;
  // class/struct/namespace/enum のブロックを避ける
  if (/\b(class|struct|namespace|enum)\b/.test(t) && t.includes("{")) {
    // ただし "class Foo { ... }" などを避けたいので false 寄り
    return false;
  }
  // 関数本体が近くにありそう
  if (t.includes("{")) return true;

  // 1〜数行後に { が来るスタイルもあるので許容
  return true;
}

/**
 * 指定位置(行/文字)から「その地点が含まれる関数定義（っぽいもの）」を取り出す（C++向け）
 */
export async function getFunctionContentFromLineAndCharacter(
  filePath: string,
  line: number,
  character: number
): Promise<string> {
  let originalFileContent = "";
  try {
    originalFileContent = await fs.readFile(filePath, "utf-8");
  } catch (e) {
    console.error(e);
    return "";
  }

  const lines = originalFileContent.split("\n");
  if (line < 0 || line >= lines.length) return "";

  // まず #define ブロックに当たってたらそれを返す（C/C++共通）
  const firstLine = lines[line] ?? "";
  if (firstLine.trimStart().startsWith("#define")) {
    const { endLine } = readDefineBlock(lines, line, 120);
    return lines.slice(line, endLine + 1).join("\n");
  }

  // 付近20行で '{' が無いなら保険で少し返す
  const failSafe = lines.slice(line, line + 25).join("\n");
  if (!failSafe.includes("{")) {
    return lines.slice(line, line + 8).join("\n");
  }

  // 「関数本体」に入るまで進む:
  // C++ はシグネチャが複数行になることが多いので、最初の '{' を探すが
  // class/namespace の '{' には引っかかりにくくする。
  const startScanState = createScanState();
  let signatureEndLine = line;
  let foundBodyOpen = false;
  let bodyOpenLine = -1;

  for (let i = line; i < Math.min(lines.length, line + 200); i++) {
    signatureEndLine = i;
    const windowText = lines.slice(line, i + 1).join("\n");

    if (!looksLikeFunctionStart(windowText)) continue;

    const { open } = scanLineForBraces(lines[i], startScanState);
    if (open > 0) {
      foundBodyOpen = true;
      bodyOpenLine = i;
      break;
    }

    // もし "){"/"){ " などが別行に来るなら、このまま進める
  }

  if (!foundBodyOpen) {
    // 最悪 fallback
    return lines.slice(line, line + 30).join("\n");
  }

  // bodyOpenLine から brace をバランスして関数本体を閉じるまで読む
  const state = createScanState();
  let openCount = 0;
  let closeCount = 0;
  const out: string[] = [];

  for (let i = line; i < lines.length; i++) {
    out.push(lines[i]);

    const { open, close } = scanLineForBraces(lines[i], state);
    // 本体開始前の '{' を拾う可能性があるので、
    // 「最初に見つけた bodyOpenLine の '{'」以降からカウントを有効化する
    if (i < bodyOpenLine) continue;

    openCount += open;
    closeCount += close;

    if (openCount > 0 && openCount === closeCount) {
      return out.join("\n");
    }

    // だらだら読むのを防ぐ上限
    if (i - line > 5000) break;
  }

  console.error("getFunctionContentFromLineAndCharacter: unbalanced braces", {
    filePath,
    line,
    character,
    openCount,
    closeCount,
  });
  return "";
}

/**
 * 正規表現用のエスケープ
 */
function escapeRegExp(s: string): string {
  return s.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

/**
 * C++ の関数名（"foo", "A::foo", "~A", "operator<<", "operator new" 等）を想定して
 * それっぽく検索するための regex を作る
 */
function buildCppFunctionSearchRegex(functionNameRaw: string): [RegExp, number][] {
  const name = functionNameRaw.trim();

  // ありがちな入力揺れ対策：
  // - "foo(" を渡される
  // - "A::foo(" を渡される
  const nameNoParen = name.endsWith("(") ? name.slice(0, -1) : name;

  // operator 系を雑に許容（operator<<, operator new, operator() など）
  const isOperator = nameNoParen.startsWith("operator");

  // :: や ~ を含む場合もある
  const escaped = escapeRegExp(nameNoParen);

  // 1) 定義っぽい:  行中に   <name> ... ( ... ) ... {  が近い
  //    ※ テンプレや戻り値後置、noexcept, const, override, requires などを許容する
  const baseName =
    isOperator
      ? // operator は空白入りもあるので "operator" 以降はゆるく
      "operator\\s*[^\\(\\n]*"
      : escaped;

  console.log("basename : ", baseName);

  const r1 =
    new RegExp(
      // 前に識別子境界（ただし :: の途中もあるので単純 \b だけにしない）
      `(^|[^\\w:~])(${baseName})(\\s*<[^;\\n{]*>)?\\s*\\(`,
      "m"
    );

  // 2) メソッド定義: A::foo(
  const r2 = new RegExp(
    `(^|[^\\w])([A-Za-z_][\\w:]\\s*[:]{0,2}\\s*)(${baseName})(\\s*<[^;\\n{]*>)?\\s*\\(`,
    "m"
  );

  // 3) コンストラクタ/デストラクタっぽい: ClassName( / ~ClassName(
  //    入力が "ClassName" や "~ClassName" だった場合に効く
  const r3 = new RegExp(
    `(^|[^\\w:])(${escaped})\\s*\\(`,
    "m"
  );

  // 4) 単純にbasenameのみの場合
  const r4 = new RegExp(
    `(^|[^\\w])(${baseName})([^\\w]|$)`,
    "m"
  );

  return [[r2, 1], [r1, 0], [r3, 1], [r4, 0]];
}

/**
 * C++ 向け: 関数名から (line, character) を探す
 * - 定義優先にしたいので、見つけた後に近傍に '{' がある候補を優先する
 * - コメント/文字列中は除外
 */
export async function getFileLineAndCharacterFromFunctionName(
  filePath: string,
  codeLine: string,
  functionName: string,
  isFirst: boolean = false,
  isSearchNameOnly: boolean = false
): Promise<[number, number]> {
  let fileContent = "";
  try {
    fileContent = await fs.readFile(filePath, "utf-8");
  } catch (e) {
    console.error(e);
    return [-1, -1];
  }

  const lines = fileContent.split("\n");
  let fixedFunctionName = functionName;
  if (!isFirst && fixedFunctionName.includes("::")) {
    fixedFunctionName = fixedFunctionName.split("::").slice(-1)[0];
  }
  if (!isFirst && fixedFunctionName.includes("->")) {
    fixedFunctionName = fixedFunctionName.split("->").slice(-1)[0];
  }
  if (!isFirst && fixedFunctionName.includes(".")) {
    fixedFunctionName = fixedFunctionName.split(".").slice(-2).join(".");
  }
  const regexes = buildCppFunctionSearchRegex(fixedFunctionName);

  // コメント/文字列中を除外するために状態を持って上からスキャン
  const state = createScanState();

  type Candidate = { line: number; ch: number; score: number };
  const candidates: Candidate[] = [];

  if (isFirst) {
    // codeLineを含む行番号を探す
    for (let i = 0; i < lines.length; i++) {
      if (lines[i].includes(codeLine.trim())) {
        // 見つかった行から関数定義を探す
        return [i, lines[i].indexOf(codeLine.trim())];
      }
    }
    return [-1, -1];
  }

  for (let i = 0; i < lines.length; i++) {
    const row = lines[i];

    // 先に状態更新しつつ、row の「有効コード部分」を取りたいが、
    // ここでは簡便に「row 全体でマッチ→後でコメント/文字列中なら捨てる」ではなく
    // brace scanner を流用して “状態だけ” を更新しつつ判定する。
    // （厳密な位置判定は難しいので、行単位でコメント/文字列中ならスキップに寄せる）
    // 行が完全に // コメントならスキップ
    if (row.trimStart().startsWith("//")) continue;

    // ブロックコメント中なら、終端が出るまでマッチさせない
    if (state.inBlockComment || state.inRawString || state.inString || state.inChar) {
      scanLineForBraces(row, state);
      continue;
    }

    // 検索
    for (const reg of regexes) {
      const r = reg[0] as RegExp;
      const regCount = reg[1] as number;
      const m = r.exec(row);
      if (!m) continue;

      // マッチ位置（ざっくり function 名の開始）
      let mindex = 0;
      for (let cnt = 0; cnt <= regCount; cnt++) {
        mindex += m[cnt+1].length;
      }
      let idx = m.index + mindex;
      if (m[0].endsWith(".")) continue; // メンバアクセスっぽいのは除外
      if (m[0].split(".").length > 2) {
        idx = idx + m[0].split(".").slice(0, -1).join(".").length; // foo.bar.baz の baz 部分だけに調整
      }

      // 定義っぽさスコア：近傍に '{' があるなら強い
      const near = lines.slice(i, i + 15).join("\n");
      let score = 0;
      if (near.includes("{")) score += 10;
      if (/;\s*$/.test(row)) score -= 5; // 宣言っぽい
      if (/\b(class|struct|namespace|enum)\b/.test(row) && row.includes("{")) score -= 8;

      candidates.push({ line: i, ch: Math.max(0, idx), score });
      break;
    }

    // 状態更新（コメント/文字列検出）
    scanLineForBraces(row, state);
  }

  if (candidates.length === 0) return [-1, -1];

  // 一番「定義っぽい」候補を返す
  candidates.sort((a, b) => b.score - a.score);
  return [candidates[0].line, candidates[0].ch];
}
