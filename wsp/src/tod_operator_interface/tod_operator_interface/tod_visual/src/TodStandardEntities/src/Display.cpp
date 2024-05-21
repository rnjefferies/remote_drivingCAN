// Copyright 2020 Feiler
#include "Display.h"

namespace TodStandardEntities {

Entity Display::create(std::shared_ptr<Scene> scene,
                       std::string name, Entity parent, const std::string& packagePath) {
    Entity display = scene->CreateEntity(name);
    auto& charMap = display.AddComponent<CharacterMapComponent>();
    display.GetComponent<TransformComponent>().setParent(parent);

    unsigned int tachoShader = ShaderSystem::createShaderProgram(
        (packagePath + "/resources/OpenGL/shaders/character.vert").c_str(),
        (packagePath + "/resources/OpenGL/shaders/character.frag").c_str());

    unsigned int numberOfLetters{ 5 };
    unsigned int numberOfVerticesPerLetter{ 4 };
    std::vector<Mesh> meshes;
    for ( unsigned int iterator=0; iterator != numberOfLetters; ++iterator ) {
        std::vector<Vertex> vertices(numberOfVerticesPerLetter);
        std::vector<unsigned int> indices{ 0, 1, 3, 1, 2, 3 };
        std::vector<Texture> textures{Texture()};
        Mesh mesh(vertices, indices, textures);
        meshes.push_back(mesh);
    }
    auto &renderable = display.AddComponent<RenderableElementComponent>(tachoShader, meshes);
    display.AddComponent<DynamicDataComponent>();

    GenerateCharactersWithTextureForDisplay(
        "/usr/share/fonts/truetype/liberation/LiberationMono-BoldItalic.ttf", charMap, renderable);

    return display;
}

void Display::onSpeedUpdate(const tod_msgs::VehicleDataConstPtr& msg, Entity &entity) {
    auto& dynamic = entity.GetComponent<DynamicDataComponent>();
    auto& renderable = entity.GetComponent<RenderableElementComponent>();
    auto& characterMap = entity.GetComponent<CharacterMapComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    dynamic.HasNewData = true;
    clearVerticesAndTextures(renderable);
    float istGeschwindigkeitFloat = 3.6f*msg->longitudinalSpeed;
    std::string istGeschwindigkeit = floatToIntString(istGeschwindigkeitFloat);
    writeTextIntoRenderable(istGeschwindigkeit, renderable, characterMap);
}

void Display::onDesiredSpeedUpdate(const tod_msgs::PrimaryControlCmdConstPtr& msg, Entity &entity) {
    auto& dynamic = entity.GetComponent<DynamicDataComponent>();
    auto& renderable = entity.GetComponent<RenderableElementComponent>();
    auto& characterMap = entity.GetComponent<CharacterMapComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    dynamic.HasNewData = true;
    clearVerticesAndTextures(renderable);
    float sollGeschwindigkeitFloat = 3.6f*msg->velocity;
    std::string sollGeschwindigkeit = floatToIntString(sollGeschwindigkeitFloat);
    sollGeschwindigkeit += " /";
    writeTextIntoRenderable(sollGeschwindigkeit, renderable, characterMap);
}


void Display::onGearUpdate(const tod_msgs::VehicleDataConstPtr& msg, Entity &entity) {
    auto& dynamic = entity.GetComponent<DynamicDataComponent>();
    auto& renderable = entity.GetComponent<RenderableElementComponent>();
    auto& characterMap = entity.GetComponent<CharacterMapComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    dynamic.HasNewData = true;
    clearVerticesAndTextures(renderable);
    std::string gearPosition = eGearPositionToString(msg->gearPosition);
    writeTextIntoRenderable(gearPosition, renderable, characterMap);
}

void Display::onDesiredGearUpdate(const tod_msgs::SecondaryControlCmdConstPtr& msg, Entity &entity) {
    auto& dynamic = entity.GetComponent<DynamicDataComponent>();
    auto& renderable = entity.GetComponent<RenderableElementComponent>();
    auto& characterMap = entity.GetComponent<CharacterMapComponent>();
    std::lock_guard<std::mutex> lock(*dynamic.Mutex);
    dynamic.HasNewData = true;
    clearVerticesAndTextures(renderable);
    std::string desiredGearPosition = eGearPositionToString(msg->gearPosition);
    desiredGearPosition += "/";
    writeTextIntoRenderable(desiredGearPosition, renderable, characterMap);
}

void Display::GenerateCharactersWithTextureForDisplay(
    const std::string &pathToTTF, CharacterMapComponent &charMap, RenderableElementComponent &renderable) {
    charMap.Characters.clear();

    FT_Library ft;
    if (FT_Init_FreeType(&ft)) {
        ROS_ERROR_STREAM("tofGL - ERROR::FREETYPE: Could not init FreeType Library");
        return;
    }

    std::string font_name = pathToTTF;
    if (font_name.empty()) {
        ROS_ERROR_STREAM("tofGL - ERROR::FREETYPE: Failed to load font_name");
        return;
    }

    FT_Face face;
    if (FT_New_Face(ft, font_name.c_str(), 0, &face)) {
        ROS_ERROR_STREAM("tofGL - ERROR::FREETYPE: Failed to load font from path " << font_name);
        return;
    }
    FT_Set_Pixel_Sizes(face, 0, (FT_UInt) charMap.PixelPerMeterRatio); // height big enough not to be pixelated

    // disable byte-alignment restriction
    RenderCommand::SetPixelStorageMode(GL_UNPACK_ALIGNMENT, 1);

    // load first 128 characters of ASCII set
    for (unsigned char c = 0; c < 128; c++) {
        if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
            std::cerr << "tofGL - ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
            continue;
        }
        // generate texture and store in character for later use
        Texture texture;
        texture.Name = "text";
        texture.Type = GL_TEXTURE_2D;
        texture.Width = face->glyph->bitmap.width;
        texture.Height = face->glyph->bitmap.rows;
        Renderer::GenerateTexture(texture, face->glyph->bitmap.buffer, renderable.ShaderProgram, 0);
        Character character = {
            texture,
            glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
            glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
            static_cast<unsigned int>(face->glyph->advance.x)
        };
        charMap.Characters.insert(std::pair<char, Character>(c, character));
    }

    FT_Done_Face(face);
    FT_Done_FreeType(ft);
}

void Display::clearVerticesAndTextures(RenderableElementComponent& renderable) {
    for ( auto& mesh : renderable.Meshes ) {
        mesh.Vertices.clear();
        mesh.Textures.clear();
    }
}

void Display::writeTextIntoRenderable(const std::string& text,
                                      RenderableElementComponent& renderable,
                                      const CharacterMapComponent& characterMap) {
    float tmpAdvance{ 0.0f };
    float scale{ 0.15f };
    float y{ 0.05f };
    for ( unsigned int index=0; index != text.size(); ++index ) {
        if ( index >= renderable.Meshes.size() ) {
            ROS_ERROR("More letters than meshes");
            continue;
        }
        auto& currentMesh = renderable.Meshes.at(index);
        Character currentChar = characterMap.Characters.at(text.at(index));
        currentMesh.Textures.emplace_back(currentChar.Tex);
        updateVertices(currentMesh, tmpAdvance, currentChar, scale, y, characterMap.PixelPerMeterRatio);
        updateAdvance(tmpAdvance, currentChar, scale);
    }
}

std::string Display::floatToIntString(const float number) {
    int numberOfDigits{4};
    if ( std::abs(number) >= 1000.0f ) {
        printf("In Display::floatToIntString(): Float to large\n");
    }
    char snumber[numberOfDigits];
    size_t sizet{ (size_t)numberOfDigits+1};
    strfromf(&snumber[0], sizet, "%.0f", number);
    std::string tempString{ snumber };
    return tempString;
}

std::string Display::eGearPositionToString(const int gearPosition) {
    std::string gearPositionString;
    switch ((eGearPosition) gearPosition) {
    case eGearPosition::GEARPOSITION_PARK :
        gearPositionString = "P";
        break;
    case eGearPosition::GEARPOSITION_REVERSE :
        gearPositionString = "R";
        break;
    case eGearPosition::GEARPOSITION_NEUTRAL :
        gearPositionString = "N";
        break;
    case eGearPosition::GEARPOSITION_DRIVE :
        gearPositionString = "D";
        break;
    default:
        gearPositionString = "P";
        break;
    }
    return gearPositionString;
}

void Display::updateVertices(Mesh& currentMesh, const float tmpAdvance,
                             const Character& currentChar, const float& scale,
                             const float& y, const float& ratioPixelPerMeter) {
    float xpos = 1.0f / ratioPixelPerMeter * (tmpAdvance + currentChar.Bearing.x * scale);
    float ypos = 1.0f / ratioPixelPerMeter * (y - (currentChar.Size.y - currentChar.Bearing.y) * scale);
    float w = 1.0f / ratioPixelPerMeter * (currentChar.Size.x * scale);
    float h = 1.0f / ratioPixelPerMeter * (currentChar.Size.y * scale);
    currentMesh.Vertices.emplace_back(glm::vec3(0.0f, -xpos, ypos+h), glm::vec2(0.0f, 0.0f), glm::vec3());
    currentMesh.Vertices.emplace_back(glm::vec3(0.0f, -xpos, ypos), glm::vec2(0.0f, 1.0f), glm::vec3());
    currentMesh.Vertices.emplace_back(glm::vec3(0.0f, -(xpos+w), ypos), glm::vec2(1.0f, 1.0f), glm::vec3());
    currentMesh.Vertices.emplace_back(glm::vec3(0.0f, -(xpos+w), ypos+h), glm::vec2(1.0f, 0.0f), glm::vec3());
}

void Display::updateAdvance(float& tmpAdvance, const Character& currentChar, const float& scale) {
    // bitshift by 6 to get value in pixels
    // (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
    tmpAdvance += (currentChar.Advance >> 6) * scale;
}

}; // namespace TodStandardEntities
