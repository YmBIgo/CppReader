import * as vscode from "vscode";
import fs from "fs/promises";

import { LinuxReader } from "./assistant";
import { Message } from "./type/Message";
import { AskResponse } from "./type/Response";
import pWaitFor from "p-wait-for";
import { LLMName } from "./llm";

let view: vscode.WebviewView | vscode.WebviewPanel;
let linuxReaderAssitant: LinuxReader | null;

export class LinuxLLMReaderProvider implements vscode.WebviewViewProvider {
  public static readonly viewType = "cpp-reader.SlidebarProvider";
  private disposables: vscode.Disposable[] = [];
  private allowedMessageType = [
    "Init",
    "InitHistory",
    "Reset",
    "Ask",
    "Clangd",
    "LinuxPath",
    "CompileCommandPath",
    "ReportPath",
    "LLMName",
    "OpenAIApiKey",
    "AnthropicApiKey",
    "PlamoApiKey",
    "GeminiApiKey",
    "OpenAIModelName",
    "AnthropicModelName",
    "GeminiModelName",
    "InitSettings",
  ];

  constructor(private readonly context: vscode.ExtensionContext) {}

  async init() {
    await linuxReaderAssitant?.doGC();
    linuxReaderAssitant = new LinuxReader(
      this.ask,
      this.say,
      this.sendError,
      this.sendState,
      ((await this.getGlobalState("llmName")) as LLMName) ?? "openai",
      // openai
      ((await this.getGlobalState("OpenAIModel")) as string) ?? "gpt-4.1",
      ((await this.getSecret("OpenAIApiKey")) as string) ?? "",
      // anthropic
      ((await this.getGlobalState("AnthropicModel")) as string) ??
        "claude-3-7-sonnet-20250219",
      ((await this.getSecret("AnthropicApiKey")) as string) ?? "",
      // plamo
      ((await this.getSecret("PlamoApiKey")) as string) ?? "",
      // gemini
      ((await this.getGlobalState("GeminiModel")) as string) ?? "",
      ((await this.getSecret("GeminiApiKey")) as string) ?? "",
      // clangd
      ((await this.getGlobalState("clangdPath")) as string) ??
        "/usr/bin/clangd",
      ((await this.getGlobalState("linuxPath")) as string) ?? "",
      ((await this.getGlobalState("compileCommand")) as string) ?? "",
      ((await this.getGlobalState("report")) as string) ?? "~/Desktop"
    );
  }

  async dispose() {
    if (view && "dispose" in view) {
      view.dispose();
    }
    while (this.disposables.length) {
      const x = this.disposables.pop();
      if (x) {
        x.dispose();
      }
    }
  }

  async say(content: string): Promise<void> {
    const sayContentJson = JSON.stringify({
      type: "say",
      say: content,
    });
    view?.webview.postMessage(sayContentJson);
  }

  async ask(content: string): Promise<AskResponse> {
    linuxReaderAssitant?.clearWebViewAskResponse();
    const askContentJson = JSON.stringify({
      type: "ask",
      ask: content,
    });
    view?.webview.postMessage(askContentJson);
    await pWaitFor(
      () => {
        return !!linuxReaderAssitant?.getWebViewAskResponse();
      },
      { interval: 500 }
    );
    const response: AskResponse = {
      ask: linuxReaderAssitant?.getWebViewAskResponse() ?? "unknown error",
    };
    console.log("response : ", response);
    return response;
  }

  async sendError(content: string) {
    const errorContentJson = JSON.stringify({
      type: "error",
      say: content,
    });
    view?.webview.postMessage(errorContentJson);
  }

  sendState(messages: Message[]): void {
    const stateContentJson = JSON.stringify({
      type: "state",
      state: messages,
    });
    view?.webview.postMessage(stateContentJson);
  }

  async resolveWebviewView(
    webviewView: vscode.WebviewView | vscode.WebviewPanel
  ) {
    view = webviewView;
    webviewView.webview.options = {
      enableScripts: true,
      localResourceRoots: [this.context.extensionUri],
    };
    webviewView.webview.html = this.getHtmlContent(webviewView.webview);

    this.init();
    this.setWebviewMessageListener(webviewView.webview);

    // Listen for when the view is disposed
    // This happens when the user closes the view or when the view is closed programmatically
    webviewView.onDidDispose(
      async () => {
        await this.dispose();
      },
      null,
      this.disposables
    );
  }

  private setWebviewMessageListener(webview: vscode.Webview) {
    webview.onDidReceiveMessage(async (message) => {
      console.log("Receive message", JSON.stringify(message));
      const type = message.type;
      if (!this.allowedMessageType.includes(type)) {
        console.log("type unknown...");
        return;
      }
      switch (type) {
        case "Init":
          const rootPath = message.rootPath ?? "";
          const rootFunctionName = message.rootFunctionName ?? "";
          const purpose = message.purpose ?? "";
          console.log("Task Start", rootPath, purpose);
          linuxReaderAssitant?.runFirstTask(
            rootPath,
            rootFunctionName,
            purpose
          );
          break;
        case "InitHistory":
          try {
            const history = await fs.readFile(message.historyPath, "utf-8");
            const historyJSON = JSON.parse(history);
            const rootPath = historyJSON.rootPath;
            const rootFunctionName = historyJSON.rootFunctionName;
            const purpose = historyJSON.purpose;
            const fixedHistoryJSON = {
              content: historyJSON.content,
              children: historyJSON.children,
            };
            // 残るのは choiceTree
            console.log("History Task Start", rootPath, purpose);
            linuxReaderAssitant?.runFirstTaskWithHistory(
              rootPath,
              rootFunctionName,
              purpose,
              fixedHistoryJSON
            );
          } catch (e) {
            console.error(e);
          }
          break;
        case "Reset":
          linuxReaderAssitant?.doGC();
          linuxReaderAssitant = null;
          linuxReaderAssitant = new LinuxReader(
            this.ask,
            this.say,
            this.sendError,
            this.sendState,
            ((await this.getGlobalState("llmName")) as LLMName) ?? "openai",
            // openai
            ((await this.getGlobalState("OpenAIModel")) as string) ?? "gpt-4.1",
            ((await this.getSecret("OpenAIApiKey")) as string) ?? "",
            // anthropic
            ((await this.getGlobalState("AnthropicModel")) as string) ??
              "claude-3-7-sonnet-20250219",
            ((await this.getSecret("AnthropicApiKey")) as string) ?? "",
            // plamo
            ((await this.getSecret("PlamoApiKey")) as string) ?? "",
            // gemini
            ((await this.getGlobalState("GeminiModel")) as string) ?? "",
            ((await this.getSecret("GeminiApiKey")) as string) ?? "",
            // clangd
            ((await this.getGlobalState("clangdPath")) as string) ??
              "/usr/bin/clangd",
            ((await this.getGlobalState("linuxPath")) as string) ?? "",
            ((await this.getGlobalState("compileCommand")) as string) ?? "",
            ((await this.getGlobalState("report")) as string) ?? "~/Desktop"
          );
          break;
        case "Ask":
          const askResponse = message.askResponse;
          console.log("receive message", askResponse);
          linuxReaderAssitant?.handleWebViewAskResponse(askResponse);
          break;
        case "Clangd":
          const clangdPath = message.text;
          this.updateGlobalState("clangdPath", clangdPath);
          this.init();
          break;
        case "LinuxPath":
          const linuxPath = message.text;
          this.updateGlobalState("linuxPath", linuxPath);
          this.init();
          break;
        case "CompileCommandPath":
          const compileCommandPath = message.text;
          this.updateGlobalState("compileCommand", compileCommandPath);
          this.init();
          break;
        case "ReportPath":
          const report = message.text;
          this.updateGlobalState("report", report);
          this.init();
          break;
        case "LLMName":
          const llmName = message.text;
          this.updateGlobalState("llmName", llmName);
          this.init();
          break;
        case "OpenAIApiKey":
          const openAIApi = message.text;
          this.storeSecret("OpenAIApiKey", openAIApi);
          this.init();
          break;
        case "AnthropicApiKey":
          const anthropicApi = message.text;
          this.storeSecret("AnthropicApiKey", anthropicApi);
          this.init();
          break;
        case "PlamoApiKey":
          const plamoApi = message.text;
          this.storeSecret("PlamoApiKey", plamoApi);
          this.init();
          break;
        case "GeminiApiKey":
          const geminiApi = message.text;
          this.storeSecret("GeminiApiKey", geminiApi);
          this.init();
          break;
        case "OpenAIModelName":
          const openAIModel = message.text;
          this.updateGlobalState("OpenAIModel", openAIModel);
          this.init();
          break;
        case "AnthropicModelName":
          const anthropicModel = message.text;
          this.updateGlobalState("AnthropicModel", anthropicModel);
          this.init();
          break;
        case "GeminiModelName":
          const geminiModel = message.text;
          this.updateGlobalState("GeminiModel", geminiModel);
          this.init();
          break;
        case "InitSettings":
          this.sendInitSettingInfoToWebView();
          break;
        default:
          break;
      }
    });
  }

  private async sendInitSettingInfoToWebView() {
    const llmName =
      ((await this.getGlobalState("llmName")) as LLMName) ?? "openai";
    // openai
    const openaiModel =
      ((await this.getGlobalState("OpenAIModel")) as string) ?? "gpt-4.1";
    const openaiApi = ((await this.getSecret("OpenAIApiKey")) as string) ?? "";
    // anthropic
    const anthropicModel =
      ((await this.getGlobalState("AnthropicModel")) as string) ??
      "claude-3-7-sonnet-20250219";
    const anthropicApi =
      ((await this.getSecret("AnthropicApiKey")) as string) ?? "";
    // plamo
    const plamoApi = ((await this.getSecret("PlamoApiKey")) as string) ?? "";
    // gemini
    const geminiModel =
      ((await this.getGlobalState("GeminiModel")) as string) ??
      "gemini-2.0-flash";
    const geminiApi = ((await this.getSecret("GeminiApiKey")) as string) ?? "";
    // clangd
    const clangd =
      ((await this.getGlobalState("clangdPath")) as string) ??
      "/usr/bin/clangd";
    const linuxPath =
      ((await this.getGlobalState("linuxPath")) as string) ?? "";
    const compileCommand =
      ((await this.getGlobalState("compileCommand")) as string) ?? "";
    const report =
      ((await this.getGlobalState("report")) as string) ?? "~/Desktop";
    view.webview.postMessage(
      JSON.stringify({
        type: "init",
        clangd,
        linuxPath,
        compileCommand,
        report,
        llmName,
        openaiApi,
        openaiModel,
        anthropicApi,
        anthropicModel,
        plamoApi,
        geminiModel,
        geminiApi,
      })
    );
  }

  private getHtmlContent(webview: vscode.Webview): string {
    // const stylesUri = getUri(webview, this.context.extensionUri, [
    //     "webview-ui",
    //     "repilot-webview",
    //     "dist",
    //     "assets",
    //     "main.css",
    // ])
    // The JS file from the React build output
    const scriptUri = getUri(webview, this.context.extensionUri, [
      "webui",
      "linux-reader-webui",
      "dist",
      "assets",
      "main.js",
    ]);
    const nonce = getNonce();

    //     <link rel="stylesheet" type="text/css" href="${stylesUri}">
    // style-src ${webview.cspSource};
    return /*html*/ `
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1,shrink-to-fit=no">
    <meta name="theme-color" content="#000000">
    <meta http-equiv="Content-Security-Policy" content="default-src 'none'; script-src 'nonce-${nonce}';">
    <title>Linux Reader</title>
  </head>
  <body>
    <noscript>You need to enable JavaScript to run this app.</noscript>
    <div id="root"></div>
    <script nonce="${nonce}" src="${scriptUri}"></script>
  </body>
</html>
`;
  }

  postMessageToWebview(message: any) {
    view?.webview.postMessage(message);
  }

  private async updateGlobalState(key: string, value: any) {
    await this.context.globalState.update(key, value);
  }

  private async getGlobalState(key: string) {
    return await this.context.globalState.get(key);
  }

  private async storeSecret(key: string, value: any) {
    await this.context.secrets.store(key, value);
  }

  private async getSecret(key: string) {
    return await this.context.secrets.get(key);
  }

  private getSimpleHTMLContent(): string {
    return `
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1,shrink-to-fit=no">
    <meta name="theme-color" content="#000000">
    <title>Linux Reader Simple</title>
  </head>
  <body>
    <div id="root">
      <h3>Linux Reader Simple</h3>
      <p>Please Input linux file path you want to search from</p>
      <input type="text" id="linux_file_path"/>
      <br/>
      <p>Please Input position in above file</p>
      <input type="text" id="linux_function_name">
      <br/>
      <input type="submit" onClick="submitLinuxInfo()"/>
      <hr/>
      <p id="linux_result_file"></p>
      <p id="linux_result_line_and_character"></p>
      <p id="linux_result"></p>
    </div>
  </body>
  <script>
    const vscode = acquireVsCodeApi();
    function submitLinuxInfo() {
        const filePath = document.getElementById("linux_file_path")?.value;
        const functionName = document.getElementById("linux_function_name")?.value;
        if (!filePath || !functionName) {
          console.log("dom not found 0")
          return;
        }
        vscode.postMessage({
            type: "textDocument/definition",
            filePath,
            functionName
        });
    }
    window.addEventListener("message", (e) => {
        const resultFileDOM = document.getElementById("linux_result_file");
        const resultLineAndCharacterDOM = document.getElementById("linux_result_line_and_character");
        const resultDOM = document.getElementById("linux_result");
        if (!resultFileDOM || !resultLineAndCharacterDOM) {
          console.log("dom not found1")
          return;
        }
        const originalMessage = typeof e === "string" ? e.data : e.data.toString();
        let parsedMessage = {}
        try {
            if (typeof originalMessage === "string") parsedMessage = JSON.parse(originalMessage);
            else if (typeof originalMessage === "object") parsedMessage = originalMessage;
            else parsedMessage =JSON.parse(originalMessage);
        } catch (e) {
            console.error(e);
        }
        const file = parsedMessage?.file ?? "unknown";
        const line_and_character = parsedMessage?.line_and_character ?? "unknown";
        const results = parsedMessage?.result;
        resultFileDOM.innerText = file
        resultLineAndCharacterDOM.innerText = line_and_character
        resultDOM.innerText = results;
    })
  </script>
</html>
`;
  }
}

export function getNonce() {
  let text = "";
  const possible =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  for (let i = 0; i < 32; i++) {
    text += possible.charAt(Math.floor(Math.random() * possible.length));
  }
  return text;
}

export function getUri(
  webview: vscode.Webview,
  extensionUri: vscode.Uri,
  pathList: string[]
) {
  return webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, ...pathList));
}
