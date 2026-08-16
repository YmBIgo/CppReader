import { generateHexString } from "../util/rand";

export type Choice = {
    functionName: string;
    functionCodeLine: string;
    originalFilePath: string;
    id: string;
    functionCodeContent?: string;
    comment?: string;
    steps?: string[];
}
export type ProcessChoice = {
    functionName: string;
    functionCodeLine: string;
    originalFilePath: string;
    comment?: string;
}
export type ChoiceTree = {
    content: Choice
    children: ChoiceTree[]
}
export type ChoicePosition = {
    depth: number;
    width: number;
}

const HISTORY_PREFIX = " ".repeat(3);

export class HistoryHandler {
    private rootPath: string;
    private choiceTree: ChoiceTree;
    private currentChoicePosition: ChoicePosition[];
    private visualizeResult: string;
    // 並列を気にしなければならなくなったら、isFirstSearchFoundを修正する 
    private isFirstSearchFound: boolean;
    constructor(rootPath: string, rootFunctionName: string, rootFunctionCodeLine: string, rootFunctionContent: string) {
        this.rootPath = rootPath;
        const rootChoice: Choice = {
            functionName: rootFunctionName,
            functionCodeLine: rootFunctionCodeLine,
            originalFilePath: rootPath,
            functionCodeContent: rootFunctionContent,
            id: generateHexString()
        };
        this.choiceTree = {
            content: rootChoice,
            children: []
        };
        this.currentChoicePosition = [{depth: 0, width: 0}];
        this.visualizeResult = "";
        this.isFirstSearchFound = false;
    }
    overWriteChoiceTree(choiceTree: ChoiceTree) {
        this.choiceTree = choiceTree;
    }
    addHistory(choices: ProcessChoice[]): ChoiceTree[] | undefined {
        console.log("choice pos", this.currentChoicePosition);
        try {
            let currentTree = this.choiceTree;
            let currentIndex = 1;
            while(true) {
                const currentWidth = this.currentChoicePosition.find((ccp) => {
                    return ccp.depth === currentIndex;
                });
                if (!currentWidth) {
                    break;
                }
                const newCurrentTree = currentTree.children[currentWidth.width];
                if (!newCurrentTree) {
                    break;
                }
                currentTree = newCurrentTree;
                currentIndex += 1;
            }
            const newChildren = choices.map((c) => {
                const newId = generateHexString();
                return {
                content: {
                    ...c,
                    id: newId,
                },
                children: []
            }});
            currentTree.children = newChildren;
            return newChildren;
        } catch (e) {
            console.error(e);
            return;
        }
    }
    choose(selectIndex: number, functionCodeContent: string, comment?: string) {
        const maxDepthPosition = this.currentChoicePosition.find((ccp) =>
            ccp.depth === this.currentChoicePosition.length - 1
        );
        if (!maxDepthPosition) {
            return;
        }
        const maxDepth = maxDepthPosition.depth;
        this.currentChoicePosition = [...this.currentChoicePosition, {
            depth: maxDepth + 1,
            width: selectIndex
        }];
        this.searchByChoicePositionArray((choiceTree) => {
            if (!functionCodeContent) return;
            choiceTree.content.functionCodeContent = functionCodeContent;
            choiceTree.content.comment = comment;
        });
    }
    private searchByChoicePositionArray(callbackFn?: (choiceTree: ChoiceTree) => void) {
        let currentTree = this.choiceTree;
        let currentDepth = 0;
        for(let choicePosition of this.currentChoicePosition) {
            const depth = choicePosition.depth;
            const width = choicePosition.width;
            if (depth !== currentDepth + 1) {
                continue;
            }
            currentTree = currentTree.children[width];
            currentDepth += 1;
        }
        if (callbackFn) {
            callbackFn(currentTree);
        }
        return currentTree;
    }
    move(selectedChoicePosition: ChoicePosition[]) {
        this.currentChoicePosition = selectedChoicePosition;
    }
    moveById(
        id: string,
        foundCallback?: (st: ChoiceTree, comment: string) => void
    ): Choice | null {
        const searchResult = this.searchTreeById(this.choiceTree, id, 0, 0, [], foundCallback);
        if (!searchResult || !searchResult.pos.length) {
            console.log(`id not found for ${id} ...`);
            return null;
        }
        this.move(searchResult.pos);
        this.isFirstSearchFound = true;
        return searchResult.processChoice;
    }
    getContentFromPos(
        pos: ChoicePosition[],
        searchChoiceTree: ChoiceTree = this.choiceTree
    ): Choice | null {
        if (pos.length === 1) {
            return searchChoiceTree.content;
        }
        const nextPos = pos[1];
        const nextWidth = nextPos.width;
        if (searchChoiceTree.children[nextWidth]) {
            return this.getContentFromPos(pos.slice(1), searchChoiceTree.children[nextWidth]);
        }
        return null;
    }
    searchTreeByIdPublic(
        id: string
    ): {pos: ChoicePosition[], processChoice: Choice, choiceTree: ChoiceTree} | null {
        const searchResult = this.searchTreeById(this.choiceTree, id, 0, 0, []);
        if (!searchResult || !searchResult.pos.length) {
            console.log(`id not found for ${id} ...`);
            return null;
        }
        return searchResult;
    }
    searchTreeById(
        searchChoiceTree: ChoiceTree,
        id: string,
        depth: number,
        width: number,
        searchPath: ChoicePosition[],
        foundCallback?: (st: ChoiceTree, comment: string) => void,
    ): {pos: ChoicePosition[], processChoice: Choice, choiceTree: ChoiceTree} | null {
        const newSearchPath = [...searchPath, {depth, width}];
        const isSame = searchChoiceTree.content.id.slice(0, 7) === id;
        if (isSame) {
            return {pos: newSearchPath, processChoice: searchChoiceTree.content, choiceTree: searchChoiceTree};
        }
        let res = null;
        searchChoiceTree.children.forEach((st, index) => {
            const result = this.searchTreeById(st, id, depth+1, index, newSearchPath, foundCallback);
            if (result) {
                res = result;
                if (foundCallback && this.isFirstSearchFound) {
                    console.log("found!", res.processChoice.functionName);
                    foundCallback(st, res.processChoice.comment ?? "");
                    this.isFirstSearchFound = false;
                }
            }
        });
        return res;
    }
    showHistory(): string {
        this.visualizeResult =`rootPath: ${this.rootPath}\n\n`;
        this.printTree(this.choiceTree);
        return this.visualizeResult;
    }
    private printTree(tree: ChoiceTree, prefix: string = "") {
        this.visualizeResult += `${prefix}|${tree.content.functionName}
${prefix}|${tree.content.id.slice(0, 7)}${tree.content.comment ? `\n${prefix}  ${tree.content.comment.slice(0, 30)}...` : ""}

`;
        for (let child of tree.children) {
            this.printTree(child, prefix + HISTORY_PREFIX);
        }
    }
    traceFunctionContent(): [string, string] {
        /* desired result is following...
            ```file_name1:function_name1
            content1...
            ```
            ```file_name2:function_name2
            content2...
            ```
         */
        let result: string = "";
        let functionResult: string = "";
        let currentTree = this.choiceTree;
        let currentDepth = 0;
        for(let index in this.currentChoicePosition) {
            const choicePosition = this.currentChoicePosition[index];
            const depth = choicePosition.depth;
            const width = choicePosition.width;
            if (depth !== currentDepth) {
                continue;
            }
            if (depth !== 0) {
                currentTree = currentTree.children[width];
            }
            const fileName = currentTree.content.originalFilePath;
            const functionName = currentTree.content?.functionName ?? currentTree.content.functionCodeLine;
            const functionCode = currentTree.content.functionCodeContent || "not provided...";
            functionResult += `${functionName} -> `;
            const currentResult = `\`\`\`${index} : ${fileName}:${functionName}
${functionCode}
\`\`\`

`;
            result += currentResult;
            currentDepth += 1;
        }
        return [result, functionResult];
    }
    getChoiceTree() {
        return this.choiceTree;
    }
    getCurrentChoicePosition() {
        return this.currentChoicePosition;
    }
}

export function traverseTree(tree: ChoiceTree, callbackFn: (choiceTree: ChoiceTree) => void) {
    callbackFn(tree);
    for (let child of tree.children) {
        traverseTree(child, callbackFn);
    }
}