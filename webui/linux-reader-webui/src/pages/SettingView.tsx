import { useEffect, useState, type Dispatch, type SetStateAction } from "react";
import { vscode } from "../utils/vscode";
import OpenAI from "openai";
import Anthropic from "@anthropic-ai/sdk";
import VscodeTextfield from "@vscode-elements/react-elements/dist/components/VscodeTextfield";
import VscodeButton from "@vscode-elements/react-elements/dist/components/VscodeButton";
import {
  VscodeOption,
  VscodeRadio,
  VscodeRadioGroup,
  VscodeSingleSelect,
} from "@vscode-elements/react-elements";

type SettingViewType = {
  setIsSettingPage: Dispatch<SetStateAction<boolean>>;
  initClangdPath: string;
  initLinuxPath: string;
  initReportPath: string;
  initCompileCommandPath: string;
  initLlmName: "openai" | "anthropic" | "plamo" | "gemini" | "";
  initOpenAIApiKey: string;
  initAnthropicApiKey: string;
  initPlamoApiKey: string;
  initGeminiApiKey: string;
  initOpenAIModelName: OpenAI.ChatModel | "";
  initAnthropicModelName: Anthropic.Model | "";
  initGeminiModelName: string;
};

const SettingView: React.FC<SettingViewType> = ({
  setIsSettingPage,
  initClangdPath,
  initLinuxPath,
  initCompileCommandPath,
  initReportPath,
  initLlmName,
  initOpenAIApiKey,
  initAnthropicApiKey,
  initPlamoApiKey,
  initGeminiApiKey,
  initOpenAIModelName,
  initAnthropicModelName,
  initGeminiModelName,
}) => {
  // clangd
  const [clangdPath, setClangdPath] = useState<string>("");
  const updateClangdPath = () => {
    vscode.postMessage({
      type: "Clangd",
      text: clangdPath,
    });
  };
  // linux path
  const [linuxPath, setLinuxPath] = useState<string>("");
  const updateLinuxPath = () => {
    vscode.postMessage({
      type: "LinuxPath",
      text: linuxPath,
    });
  };
  // compile commands json path
  const [compileCommandPath, setCompileCommandPath] = useState<string>("");
  const updateCompipleCommandPath = () => {
    vscode.postMessage({
      type: "CompileCommandPath",
      text: compileCommandPath,
    });
  };
  // report path
  const [reportPath, setReportPath] = useState<string>("");
  const updateReportPath = () => {
    vscode.postMessage({
      type: "ReportPath",
      text: reportPath,
    });
  };
  // llm name
  const [llmName, setLlmName] = useState<
    "openai" | "anthropic" | "plamo" | "gemini" | ""
  >("");
  const updateLlmName = () => {
    vscode.postMessage({
      type: "LLMName",
      text: llmName,
    });
  };
  // openai api key
  const [openAIApiKey, setOpenAIApiKey] = useState<string>("");
  const updateOpenAIApiKey = () => {
    vscode.postMessage({
      type: "OpenAIApiKey",
      text: openAIApiKey,
    });
  };
  // anthropic api key
  const [anthropicApiKey, setAnthropicApiKey] = useState<string>("");
  const updateAnthropicApiKey = () => {
    vscode.postMessage({
      type: "AnthropicApiKey",
      text: anthropicApiKey,
    });
  };
  // plamo api key
  const [plamoApiKey, setPlamoApiKey] = useState<string>("");
  const updatePlamoApiKey = () => {
    vscode.postMessage({
      type: "PlamoApiKey",
      text: plamoApiKey,
    });
  };
  // gemini api key
  const [geminiApiKey, setGeminiApiKey] = useState<string>("");
  const updateGeminiApiKey = () => {
    vscode.postMessage({
      type: "GeminiApiKey",
      text: geminiApiKey,
    });
  };
  // openai model name
  const [openAIModelName, setOpenAIModelName] = useState<OpenAI.ChatModel | "">(
    ""
  );
  const [openAIModelList] = useState<string[]>([
    "gpt-5.6-sol",
    "gpt-5.6-terra",
    "gpt-5.6-luna",
    "gpt-5.5",
    "gpt-5.4",
    "gpt-5.2",
    "gpt-5.1",
    "gpt-5",
    "gpt-4.1",
    "gpt-4o",
    "gpt-4o-mini",
    "o1",
    "o3",
    "o4-mini",
    "o3-mini",
  ]);
  const updateOpenAIModelName = () => {
    vscode.postMessage({
      type: "OpenAIModelName",
      text: openAIModelName,
    });
  };
  // anthropic model name
  const [anthropicModelName, setAnthropicModelName] = useState<
    Anthropic.Model | ""
  >("");
  const [anthropicModelList] = useState<string[]>([
    "claude-sonnet-4-5",
    "claude-haiku-4-5",
    "claude-opus-4-1",
    "claude-sonnet-4-20250514",
    "claude-opus-4-20250514",
    "claude-3-7-sonnet-20250219",
    "claude-3-5-sonnet-20241022",
  ]);
  const updateAnthropicModelName = () => {
    vscode.postMessage({
      type: "AnthropicModelName",
      text: anthropicModelName,
    });
  };
  // gemini model name
  const [geminiModelName, setGeminiModelName] = useState<string>("");
  const [geminiModelList] = useState<string[]>([
    "gemini-2.5-pro-preview-06-05",
    "gemini-2.5-flash-preview-05-20",
    "gemini-2.0-flash",
    "gemini-2.0-flash-lite",
  ]);
  const updateGeminiModelName = () => {
    vscode.postMessage({
      type: "GeminiModelName",
      text: geminiModelName,
    });
  };

  useEffect(() => {
    vscode.postMessage({
      type: "InitSettings",
    });
  }, []);

  useEffect(() => {
    setClangdPath(initClangdPath);
  }, [initClangdPath]);
  useEffect(() => {
    setLinuxPath(initLinuxPath);
  }, [initLinuxPath]);
  useEffect(() => {
    setCompileCommandPath(initCompileCommandPath);
  }, [initCompileCommandPath]);
  useEffect(() => {
    setReportPath(initReportPath);
  }, [initReportPath]);
  useEffect(() => {
    setLlmName(initLlmName);
  }, [initLlmName]);
  useEffect(() => {
    setOpenAIApiKey(initOpenAIApiKey);
  }, [initOpenAIApiKey]);
  useEffect(() => {
    setAnthropicApiKey(initAnthropicApiKey);
  }, [initAnthropicApiKey]);
  useEffect(() => {
    setPlamoApiKey(initPlamoApiKey);
  }, [initPlamoApiKey]);
  useEffect(() => {
    setGeminiApiKey(initGeminiApiKey);
  }, [initGeminiApiKey]);
  useEffect(() => {
    setOpenAIModelName(initOpenAIModelName);
  }, [initOpenAIModelName]);
  useEffect(() => {
    setAnthropicModelName(initAnthropicModelName);
  }, [initAnthropicModelName]);
  useEffect(() => {
    setGeminiModelName(initGeminiModelName);
  }, [initGeminiModelName]);
  return (
    <div
      style={{
        width: "350px",
        height: "95vh",
        overflow: "scroll",
        position: "relative",
        borderRight: "1px solid black",
      }}
      id="settingsContainer"
    >
      <h3>設定</h3>
      <hr />
      <p>Clangd のパス</p>
      <VscodeTextfield
        value={clangdPath}
        onChange={(e) =>
          setClangdPath(
            (e?.target as HTMLTextAreaElement)?.value ?? "error occurs"
          )
        }
      />
      <br />
      <VscodeButton onClick={updateClangdPath}>Save Clangd Path</VscodeButton>
      <hr />
      <p>Linux のパス</p>
      <VscodeTextfield
        value={linuxPath}
        onChange={(e) =>
          setLinuxPath(
            (e?.target as HTMLTextAreaElement)?.value ?? "error occurs"
          )
        }
      />
      <br />
      <VscodeButton onClick={updateLinuxPath}>Save Linux Path</VscodeButton>
      <hr />
      <p>Compile Commands JSON のパス</p>
      <VscodeTextfield
        value={compileCommandPath}
        onChange={(e) =>
          setCompileCommandPath(
            (e?.target as HTMLTextAreaElement)?.value ?? "error occurs"
          )
        }
      />
      <br />
      <VscodeButton onClick={updateCompipleCommandPath}>
        Save Compile Commands JSON Path
      </VscodeButton>
      <hr />
      <p>レポートを保存する先のパス</p>
      <VscodeTextfield
        value={reportPath}
        onChange={(e) =>
          setReportPath(
            (e?.target as HTMLTextAreaElement)?.value ?? "error occurs"
          )
        }
      />
      <br />
      <VscodeButton onClick={updateReportPath}>Save Report Path</VscodeButton>
      <hr />
      <p>使用するLLMを選んでください (OpenAI / Anthropic / Plamo)</p>
      <VscodeRadioGroup
        onChange={(e) => {
          const value = (e.target as HTMLInputElement)?.value ?? "";
          if (!["openai", "anthropic", "plamo", "gemini"].includes(value))
            return;
          setLlmName(value as "openai" | "anthropic" | "plamo" | "gemini");
        }}
      >
        <VscodeRadio value="openai" label="openai">
          OpenAI
        </VscodeRadio>
        <VscodeRadio value="anthropic" label="anthropic">
          Anthropic
        </VscodeRadio>
        <VscodeRadio value="plamo" label="plamo">
          Plamo
        </VscodeRadio>
        <VscodeRadio value="gemini" label="gemini">
          Gemini
        </VscodeRadio>
      </VscodeRadioGroup>
      <br />
      <VscodeButton disabled={!llmName} onClick={updateLlmName}>
        Save LLM
      </VscodeButton>
      <hr />
      {llmName === "" ? (
        <p>LLMを選んでください</p>
      ) : llmName === "openai" ? (
        <>
          <p>OpenAIのAPIキー</p>
          <VscodeTextfield
            value={openAIApiKey}
            type="password"
            onChange={(e) =>
              setOpenAIApiKey(
                (e?.target as HTMLTextAreaElement)?.value ?? "error occurs"
              )
            }
          />
          <br />
          <VscodeButton onClick={updateOpenAIApiKey}>
            Save OpenAI API Key
          </VscodeButton>
          <hr />
          <p>OpenAIのモデル</p>
          <VscodeSingleSelect
            onChange={(e) => {
              const value = (e.target as HTMLSelectElement).value;
              if (!openAIModelList.includes(value)) return;
              setOpenAIModelName(value as OpenAI.ChatModel);
            }}
            value={openAIModelName}
          >
            {openAIModelList.map((model) => {
              return <VscodeOption description={model}>{model}</VscodeOption>;
            })}
          </VscodeSingleSelect>
          <br />
          <VscodeButton onClick={updateOpenAIModelName}>
            Save OpenAI Model
          </VscodeButton>
          <hr />
        </>
      ) : llmName === "anthropic" ? (
        <>
          <p>AnthropicのAPIキー</p>
          <VscodeTextfield
            value={anthropicApiKey}
            type="password"
            onChange={(e) =>
              setAnthropicApiKey(
                (e?.target as HTMLTextAreaElement)?.value ?? "error occurs"
              )
            }
          />
          <br />
          <VscodeButton onClick={updateAnthropicApiKey}>
            Save Anthropic API Key
          </VscodeButton>
          <hr />
          <p>Anthropicのモデル</p>
          <VscodeSingleSelect
            onChange={(e) => {
              const value = (e.target as HTMLSelectElement).value;
              if (!anthropicModelList.includes(value)) return;
              setAnthropicModelName(value as Anthropic.Model);
            }}
            value={anthropicModelName}
          >
            {anthropicModelList.map((model) => {
              return <VscodeOption description={model}>{model}</VscodeOption>;
            })}
          </VscodeSingleSelect>
          <br />
          <VscodeButton onClick={updateAnthropicModelName}>
            Save Anthropic Model
          </VscodeButton>
          <hr />
        </>
      ) : llmName === "plamo" ? (
        <>
          <p>PlamoのAPIキー</p>
          <VscodeTextfield
            value={plamoApiKey}
            type="password"
            onChange={(e) =>
              setPlamoApiKey(
                (e?.target as HTMLTextAreaElement)?.value ?? "error occurs"
              )
            }
          />
          <br />
          <VscodeButton onClick={updatePlamoApiKey}>
            Save Plamo API Key
          </VscodeButton>
          <hr />
        </>
      ) : llmName === "gemini" ? (
        <>
          <p>GeminiのAPIキー</p>
          <VscodeTextfield
            value={geminiApiKey}
            type="password"
            onChange={(e) => {
              setGeminiApiKey(
                (e?.target as HTMLTextAreaElement)?.value ?? "error occurs"
              );
            }}
          />
          <br />
          <VscodeButton onClick={updateGeminiApiKey}>
            Save Gemini API Key
          </VscodeButton>
          <hr />
          <p>Geminiのモデル</p>
          <VscodeSingleSelect
            onChange={(e) => {
              const value = (e.target as HTMLSelectElement).value;
              if (!geminiModelList.includes(value)) return;
              setGeminiModelName(value);
            }}
            value={geminiModelName}
          >
            {geminiModelList.map((model) => {
              return <VscodeOption description={model}>{model}</VscodeOption>;
            })}
          </VscodeSingleSelect>
          <br />
          <VscodeButton onClick={updateGeminiModelName}>
            Save Gemini Model
          </VscodeButton>
          <hr />
        </>
      ) : (
        <p>不明なLLM</p>
      )}
      <VscodeButton onClick={() => setIsSettingPage(false)}>
        Chatに戻る
      </VscodeButton>
    </div>
  );
};

export default SettingView;
