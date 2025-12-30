import * as assert from 'assert';
import {
    getFunctionContentFromLineAndCharacter,
    getFileLineAndCharacterFromFunctionName
} from "../core/lsp";
import lidar_markser_localizer from "./stub/lsp/lidar_markser_localizer.json";
import path from "path";

// please edit pathToYourDirectory when you want to test it.
const pathToYourDirectory = "/Users/kazuyakurihara/Documents/open_source/car/CppReader"

suite('Extension LSP', () => {
    // getFunctionContentFromLineAndCharacter
    // 行数・何文字目・ファイルパスから、関数の内容を取得
    // fork.c
	test('getFunctionContentFromLineAndCharacter lidar_marker_localizer.cpp', async() => {
        const stubFilePath = path.resolve(pathToYourDirectory, "src", "test", "stub", "lsp", "lidar_marker_localizer.cpp");
        for(let i = 0; i < lidar_markser_localizer.length; i++) {
            const currentFileContent = lidar_markser_localizer[i];
            if (!currentFileContent.content) continue;
            const functionContent = await getFunctionContentFromLineAndCharacter(
                stubFilePath,
                currentFileContent.line,
                currentFileContent.character
            );
            assert.strictEqual(functionContent, currentFileContent.content);
            // console.log("PASSED : getFunctionContentFromLineAndCharacter @ ", currentFileContent.functionName);
        }
    });

    // getFileLineAndCharacterFromFunctionName
    // 関数の先頭１行目とファイルパスから、行数・何文字目かを取得
    // fork.c
    test('getFileLineAndCharacterFromFunctionName lidar_marker_localizer.cpp', async () => {
        const stubFilePath = path.resolve(pathToYourDirectory, "src", "test", "stub", "lsp", "lidar_marker_localizer.cpp");
        for(let i = 0; i < lidar_markser_localizer.length; i++) {
            const currentFileContent = lidar_markser_localizer[i];
            const [line, character] = await getFileLineAndCharacterFromFunctionName(
                stubFilePath,
                currentFileContent.firstLine,
                currentFileContent.functionName
            )
            assert.strictEqual(currentFileContent.line, line);
            assert.strictEqual(currentFileContent.character, character);
            // console.log("PASSED : getFileLineAndCharacterFromFunctionName @ ", currentFileContent.functionName);
        }
    });
    test('getFileLineAndCharacterFromFunctionName lidar_marker_localizer.cpp isFirst', async () => {
        const stubFilePath = path.resolve(pathToYourDirectory, "src", "test", "stub", "lsp", "lidar_marker_localizer.cpp");
        for(let i = 0; i < lidar_markser_localizer.length; i++) {
            const currentFileContent = lidar_markser_localizer[i];
            if (!currentFileContent.queryLine || !currentFileContent.queryCharacter) continue;
            const [line, character] = await getFileLineAndCharacterFromFunctionName(
                stubFilePath,
                currentFileContent.firstLine,
                currentFileContent.firstLine,
                true
            )
            assert.strictEqual(currentFileContent.queryLine, line);
            assert.strictEqual(currentFileContent.queryCharacter, character);
            // console.log("PASSED : getFileLineAndCharacterFromFunctionName @ ", currentFileContent.functionName);
        }
    });
});